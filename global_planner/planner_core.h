// planner_core.h
//
// Standalone global-routing path planner.
//
// Pure geometry: NO KiCad / PNS / wx dependency. Only the C++ standard library.
// The planner turns a set of obstacles + a (start, target) pair into a small set
// of ranked candidate "homotopy" paths (which side of each obstacle to pass),
// expressed as polyline waypoints.
//
// It does NOT route or shove. The caller feeds the waypoints to a detailed router
// (e.g. KiCad PNS) and reports failures back via bumpCongestion() so the planner
// can re-rank (negotiated-congestion / PathFinder style).
//
// Units are arbitrary but must be consistent (mm or nm — your choice).

#pragma once

#include <vector>
#include <cstdint>

namespace gplan {

struct Point
{
    double x = 0.0;
    double y = 0.0;
};

// A closed polygon (vertices in order; first/last need not be repeated).
using Polygon = std::vector<Point>;

struct Obstacle
{
    Polygon poly;
    bool    fixed = true;   // true  = pad / locked track / keepout / board edge  -> HARD
                            // false = movable copper (existing routed track/via) -> SOFT
};

struct PlannerParams
{
    double trackWidth = 0.15;   // width of the net we are routing
    double clearance  = 0.15;   // required clearance to neighbours

    int    kPaths     = 5;      // how many distinct candidate paths to return

    double wCongestion = 1.0;   // weight of the "channel is filling up" penalty
    double wTightness  = 1.0;   // weight of the "this gap is narrow" penalty

    double reusePenalty = 3.0;  // edge cost multiplier to force later paths to differ

    double cornerOffset = 1.10; // how far (× margin) to push graph nodes outside hulls
};

struct Path
{
    std::vector<Point> waypoints;   // start ... corners ... target
    double             cost = 0.0;  // planner cost (NOT a DRC guarantee)
};

// A localized congestion "bump" injected by the caller after a PNS failure.
// Edges passing within `radius` of `center` get their congestion scaled by `factor`.
struct CongestionBump
{
    Point  center;
    double radius = 0.0;
    double factor = 1.0;
};

class Planner
{
public:
    // NOTE: for v1, FIXED obstacle polygons must be CONVEX (e.g. PNS Hull()).
    // Movable obstacles may be any shape (used only for cost, never blocking).
    Planner( std::vector<Obstacle> obstacles, PlannerParams params );

    // Compute up to params.kPaths distinct candidate paths from start to target.
    // Returned in ascending cost order. Empty if start/target are unreachable
    // through the FIXED obstacles (movable obstacles never block, only cost).
    std::vector<Path> plan( Point start, Point target );

    // PathFinder feedback: tell the planner that routing failed near `where`
    // (e.g. r.blockingObstacle position from PNS). Future plan() calls will avoid
    // that area. Call repeatedly to escalate.
    void bumpCongestion( Point where, double radius, double factor );

    void clearCongestion();

private:
    struct Edge    { int to; int id; };          // id -> m_edgeList (holds the weight)
    struct EdgeRef { int a; int b; double weight; };

    // Build the node list (start, target, offset fixed-hull corners).
    void buildNodes( const Point& start, const Point& target );
    // (Re)build adjacency + weights for the current node set.
    void buildEdges();
    // A* over the current graph with a per-edge multiplier (for distinct paths).
    std::vector<int> aStar( int src, int dst, const std::vector<double>& edgeMul ) const;

    double edgeWeight( const Point& a, const Point& b ) const;
    bool   edgeBlocked( const Point& a, const Point& b ) const; // fixed-obstacle test

    std::vector<Obstacle>          m_obstacles;
    PlannerParams                  m_params;
    std::vector<CongestionBump>    m_bumps;

    // Precomputed in the constructor (configuration-space expansion).
    std::vector<Polygon>           m_fixedOrig;       // for tightness/capacity
    std::vector<Polygon>           m_fixedInflated;   // offset by margin -> graph nodes
    std::vector<Polygon>           m_fixedBlock;      // offset by ~margin -> edge blocking
    std::vector<Polygon>           m_movable;         // for congestion (cost only)

    // Graph state, rebuilt per plan().
    std::vector<Point>             m_nodes;          // index 0 = start, 1 = target
    std::vector<std::vector<Edge>> m_adj;            // m_adj[i] = edges out of node i
    std::vector<EdgeRef>           m_edgeList;       // flat list; index == edge id
};

} // namespace gplan
