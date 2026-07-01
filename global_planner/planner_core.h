// planner_core.h
//
// Standalone global-routing path planner (v2: multi-layer + vias).
//
// Pure geometry: NO KiCad / PNS / wx dependency. Only the C++ standard library.
// Turns a set of obstacles + a (start, target) pair — each on a copper layer —
// into a small set of ranked candidate "homotopy" paths (which side of each
// obstacle to pass, and where to change layers), expressed as polyline waypoints
// that carry a layer. A layer change between two same-position waypoints means
// "drop a via here".
//
// It does NOT route or shove. The caller feeds the waypoints to a detailed router
// (KiCad PNS) and reports failures back via bumpCongestion().
//
// Units are arbitrary but must be consistent (mm or nm). Layers are integer ids;
// PlannerParams::layers lists them in physical stack order (adjacent entries may
// be connected by a via).

#pragma once

#include <mutex>
#include <vector>

namespace gplan {

struct Point
{
    double x = 0.0;
    double y = 0.0;
};

using Polygon = std::vector<Point>;

struct Obstacle
{
    Polygon poly;
    bool    fixed = true;   // true = pad/locked/keepout (HARD), false = movable copper (SOFT)
    int     layer = 0;      // copper layer this obstacle sits on.
                            // A multi-layer item (via / THT pad) is added once PER layer.
};

struct PlannerParams
{
    double trackWidth = 0.15;
    double clearance  = 0.15;

    int    kPaths     = 5;

    double wCongestion = 1.0;
    double wTightness  = 1.0;
    double reusePenalty = 3.0;

    // Layer stack in physical order, e.g. {0, 1} or {0, 1, 2, 3}. Vias may connect
    // entries that are adjacent in THIS list.
    std::vector<int> layers = { 0 };

    double viaCost      = 5.0;   // cost added per layer transition (steer via count)
    double viaClearance = 0.15;  // clearance required around a via landing site
    double viaDiameter  = 0.45;  // via pad diameter (for the landing-site clear test)
};

struct Waypoint
{
    Point p;
    int   layer = 0;
};

struct Path
{
    std::vector<Waypoint> waypoints;   // start ... corners/vias ... target
    double                cost = 0.0;  // planner cost (NOT a DRC guarantee)
};

struct CongestionBump
{
    Point  center;
    double radius = 0.0;
    double factor = 1.0;
};

class Planner
{
public:
    // NOTE: FIXED obstacle polygons must be CONVEX (e.g. PNS Hull()).
    // Movable obstacles may be any shape (used only for cost, never blocking).
    Planner( std::vector<Obstacle> obstacles, PlannerParams params );

    // Up to params.kPaths distinct candidate paths, ascending cost.
    std::vector<Path> plan( Point start, int startLayer, Point target, int targetLayer );

    // Convenience for single-layer use (start/target on params.layers[0]).
    std::vector<Path> plan( Point start, Point target );

    // PathFinder feedback after a router failure near `where` (any layer).
    void bumpCongestion( Point where, double radius, double factor );
    void clearCongestion();

private:
    struct Node { Point p; int layer; };
    struct Edge { int to; int id; };
    struct EdgeRef { int a; int b; double weight; };

    void  buildLayers();                               // group obstacles per layer
    void  buildNodes( Point start, int sL, Point target, int tL );
    void  buildEdges();
    std::vector<int> aStar( int src, int dst, const std::vector<double>& mul ) const;

    double edgeWeight( Point a, Point b, int layer ) const;
    bool   edgeBlocked( Point a, Point b, int layer ) const;
    bool   viaSiteClear( Point p, int layer ) const;
    int    stackPos( int layer ) const;                // index of layer in params.layers

    std::vector<Obstacle>       m_obstacles;
    PlannerParams               m_params;
    std::vector<CongestionBump> m_bumps;
    // Guards the per-instance mutable state (m_bumps + the graph rebuilt in
    // plan()). One Planner per worker is still the recommended pattern; this
    // makes plan()/bumpCongestion safe to call concurrently on ONE instance.
    mutable std::mutex          m_mutex;
    // Reusable scratch buffer for SpatialGrid::queryInto results — avoids a
    // heap allocation on every edgeBlocked/edgeWeight/viaSiteClear call.
    mutable std::vector<int>    m_queryBuf;

    struct AABB { double x0, y0, x1, y1; };   // T5: per-hull bounding box for culling

    // T-GRID — uniform spatial hash over hull AABBs. edgeBlocked/edgeWeight/
    // viaSiteClear/insideAnyBlock used to scan ALL m hulls per candidate edge
    // (O(n^2 * m) total for the O(n^2) edge candidates alone) — the AABB check
    // (T5) only skips the expensive polygon test, not the scan itself. A grid
    // query returns just the hulls near the segment/point (O(k), k << m for
    // spread-out PCB layouts), cutting graph build to near O(n^2 * k). Built
    // once per obstacle set in buildLayers(); read-only after (safe under the
    // mutex plan() already holds for the whole rebuild).
    struct SpatialGrid
    {
        double cell = 1.0, ox = 0.0, oy = 0.0;
        int    nx = 1, ny = 1;
        std::vector<std::vector<int>> cells;          // cells[cy*nx+cx] -> hull indices
        mutable std::vector<int> epoch;                // per-hull last-seen query id (dedup)
        mutable int epochCounter = 0;

        void build( const std::vector<AABB>& boxes );
        // Appends every hull index whose AABB may overlap [x0,y0]-[x1,y1] into
        // `out`, each at most once (per-grid epoch dedup, no per-query alloc).
        void queryInto( double x0, double y0, double x1, double y1,
                        std::vector<int>& out ) const;

    private:
        int cellX( double x ) const;
        int cellY( double y ) const;
    };

    // Per-layer configuration-space data (keyed by layer id).
    struct LayerData
    {
        std::vector<Polygon> fixedOrig;     // for tightness/capacity
        std::vector<Polygon> fixedInflated; // offset by margin -> graph nodes
        std::vector<Polygon> fixedBlock;    // offset by ~margin -> edge blocking
        std::vector<Polygon> movable;       // for congestion
        // Parallel AABBs (T5 spatial cull): index-aligned with the lists above.
        std::vector<AABB>    fixedOrigBox, fixedBlockBox, movableBox;
        // fixedGrid indexes fixedBlockBox (the OUTER of the two fixed boxes —
        // fixedBlock is fixedOrig offset outward by margin-eps, so
        // fixedBlockBox ⊇ fixedOrigBox — same index space, one grid serves both
        // fixedOrig and fixedBlock queries; the existing per-hull AABB check
        // still runs after, so this can only add candidates, never drop valid
        // ones). movableGrid indexes movableBox separately.
        SpatialGrid fixedGrid, movableGrid;
    };
    std::vector<LayerData> m_layerData;      // indexed by stackPos()

    // Graph state, rebuilt per plan().
    std::vector<Node>              m_nodes;
    std::vector<std::vector<Edge>> m_adj;
    std::vector<EdgeRef>           m_edgeList;
    int                            m_srcIdx = 0;
    int                            m_dstIdx = 1;
};

} // namespace gplan
