// pns_bridge.h
//
// KiCad-LINKED adapter that connects the standalone planner core (gplan) to
// KiCad's PNS detailed router. This is the half that CANNOT be standalone:
// it reads a real BOARD and drives PNS push-and-shove.
//
//   - getObstacles(layer): BOARD -> std::vector<gplan::Obstacle>
//   - routeAndCheck(path):  gplan waypoints -> PNS shove -> (ok, blocking point)
//
// Build: inside the KiCad source tree (links pcbnew/kicommon). It is NOT part of
// the dependency-free core and will not compile in isolation. Modelled on the
// verified headless harness qa/tools/pns/pns_log_player.cpp and the evaluate
// path in pcbnew/router/pns_router.cpp (markViolations / QueryColliding).

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "planner_core.h"

class BOARD;
class SETTINGS_MANAGER;
namespace PNS { class ROUTER; class ITEM; class ROUTING_SETTINGS; }
class PNS_KICAD_IFACE_BASE;

namespace gbridge {

struct RouteResult
{
    bool        ok        = false;   // placed and reached, no residual collisions
    bool        collided  = false;   // placed but obstacles remain
    bool        placed    = false;   // PNS produced any geometry at all
    int         vias      = 0;       // vias the route placed (layer changes)
    gplan::Point blocking;           // where it got stuck (for bumpCongestion)
    std::string reason;              // router->FailureReason()
};

// Routed geometry extracted from a successful PNS route (board units = nm).
// Layers are KiCad PCB_LAYER_ID ints. The host turns these into PCB_TRACK /
// PCB_VIA on the board; nothing here mutates the BOARD.
struct RouteGeom
{
    bool ok        = false;
    bool collided  = false;
    bool placed    = false;
    int  vias      = 0;
    int  netcode   = -1;
    std::string reason;
    // ADDED items (the new route + shoved neighbours at their NEW positions).
    // each seg: { x1, y1, x2, y2, width, boardLayer }
    std::vector<std::vector<double>> segs;
    std::vector<std::string>         segNets;   // parallel to segs (net name)
    // each via: { x, y, diameter, drill, boardLayerTop, boardLayerBottom }
    std::vector<std::vector<double>> viaList;
    std::vector<std::string>         viaNets;   // parallel to viaList (net name)
    // REMOVED items (shoved neighbours at their OLD positions — the host must
    // delete these from the board so the shove is realized, not duplicated).
    std::vector<std::vector<double>> removedSegs;   // x1,y1,x2,y2,width,boardLayer
    std::vector<std::vector<double>> removedVias;   // x,y,boardLayerTop,boardLayerBottom
};

// LOSSLESS commit-to-world result. Unlike RouteGeom (a post-hoc geometry diff
// that delete+re-adds shoved neighbours and loses via/connectivity links), this
// is PNS's OWN parent-matched change stream from CommitRouting(): shoved
// neighbours come back as MODIFY-by-UUID (host moves the existing board item in
// place, keeping its identity + via links), and the route is committed into the
// PNS world so the next net shoves against it (route-order).
struct RouteChange
{
    bool ok        = false;   // reached the target cleanly AND committed to world
    bool placed    = false;   // PNS produced copper (may NOT reach — long-haul fragment)
    bool reached   = false;   // the head actually reached the target point+layer
    bool collided  = false;
    int  vias      = 0;
    int  netcode   = -1;
    gplan::Point blocking;    // farthest point reached toward target when !reached —
                              // insert an intermediate waypoint here and retry
    std::string reason;
    // ADDED — new copper to create on the board.
    std::vector<std::vector<double>> addedSegs;   // {x1,y1,x2,y2,width,boardLayer}
    std::vector<std::vector<double>> addedVias;   // {x,y,dia,drill,boardTop,boardBot}
    // MODIFIED — shoved neighbours: modify the EXISTING board item with this uuid
    // to the new geometry (do NOT delete+re-add — that breaks via connectivity).
    std::vector<std::string>         modSegUuids;
    std::vector<std::vector<double>> modSegs;     // parallel; {x1,y1,x2,y2,width,boardLayer}
    std::vector<std::string>         modViaUuids;
    std::vector<std::vector<double>> modVias;      // parallel; {x,y,dia,drill,boardTop,boardBot}
    // REMOVED — existing board items to delete (by uuid).
    std::vector<std::string>         removedUuids;
};

// Diagnostic for a routing TARGET point (catches "aimed at a pad centre in a
// congested row" mistakes). Lets the host flag/re-target before routing.
struct TargetProbe
{
    bool   seedable        = false;   // an item of `net` exists at the point on this layer
    bool   congested       = false;   // foreign copper within `clearance` of the point
    double nearestForeign  = -1.0;    // nm to nearest foreign item (-1 = none in window)
    int    foreignNet      = -1;      // net code of that foreign item
    double nearestOther    = -1.0;    // nearest foreign on the adjacent layer (escape hint)
};

// Speculative component-drag probe (tries a move WITHOUT committing).
struct DragProbe
{
    bool   clean = false;   // the move resolves with no residual collision
    double cost  = -1.0;    // total length of the reshaped connected tracks (less = better)
    int    shoved = 0;      // number of track segments the move disturbed
};

// Result of a router-driven component-placement search (probe candidates, commit best).
struct ShoveResult
{
    bool        committed = false;   // a clean candidate was found AND committed
    double      x = 0.0, y = 0.0;    // chosen new position
    double      cost = -1.0;
    RouteChange change;              // the committed change stream (when committed)
};

// §4.6 — result of optimizeRoute: PNS::OPTIMIZER post-route pass over ONE
// assembled line (the joint-to-joint run of copper under the probe point).
struct OptimizeResult
{
    bool        ok       = false;   // pass ran cleanly (line found, result collision-free)
    bool        found    = false;   // copper found at the probe point
    bool        improved = false;   // optimizer changed the geometry (then committed)
    double      lengthBefore = 0.0; // nm — assembled line before / after the pass
    double      lengthAfter  = 0.0;
    int         cornersBefore = 0;  // line vertex count before / after
    int         cornersAfter  = 0;
    std::string reason;
    RouteChange change;             // committed change stream (same contract as routeAndCommit)
};

// T-CORRIDOR — result of clearEscapeCorridor: the cascade of relocations it
// committed while trying to open a straight probe from the escape point
// toward the target direction.
struct EscapeClearResult
{
    bool                     ok         = false;   // final probe succeeded
    int                      iterations = 0;       // relocation attempts made
    std::vector<RouteChange> moves;                // one per committed relocation, in order
    gplan::Point             blocking;              // final blocked point (only meaningful if !ok)
};

class PnsBridge
{
public:
    PnsBridge();
    ~PnsBridge();

    // Loads board + project (.kicad_pro) + custom rules (.kicad_dru) by the
    // usual filename convention, builds the DRC engine, and syncs the PNS world.
    // Returns false on failure. (Links BOARD_LOADER -> pcbnew kiface objects.)
    // If aErr is non-null, it is filled with a specific reason on failure:
    // "project load failed: ...", "board parse failed: ...", "empty board", or
    // "attach failed" (0.6 — distinguishes file/parse/DRC/attach failure modes).
    bool load( const std::string& pcbPath, std::string* aErr = nullptr );

    // Routing setup against an already-loaded BOARD (its DRC engine must already
    // be initialized). Does NOT depend on BOARD_LOADER — use when the host has
    // its own board-loading path. Builds the iface/router and syncs the world.
    // Re-callable: tears down any previous attach first (T10).
    bool attach( BOARD* board );

    // T10 — release the router/iface/settings so the bridge can be re-attached
    // to another board in the same process. Called automatically by attach().
    void cleanup();

    // T12 — PNS routing mode. Values mirror PNS_MODE; mapped in the .cpp so the
    // PNS header stays out of this public header. Default = Shove (unchanged).
    enum class RouteMode { MarkObstacles = 0, Shove = 1, Walkaround = 2 };
    void setMode( RouteMode mode );

    // Map a KiCad copper layer (PCB_LAYER_ID int) to/from a PNS layer index.
    int  pnsLayer( int boardLayer ) const;

    // Extract obstacles on one PNS copper layer as gplan::Obstacle:
    //   fixed = pads / locked tracks / keepouts ; movable = unlocked copper.
    // Fixed shapes are emitted as (convex) bounding boxes for v1 robustness.
    std::vector<gplan::Obstacle> getObstacles( int pnsLayer ) const;

    // Convenience: obstacles across all copper layers (each tagged with its layer).
    std::vector<gplan::Obstacle> getAllObstacles() const;

    // Drive PNS along the planned waypoints (shove mode) and report the outcome.
    // waypoints carry layers; a layer change is realized as a via (see .cpp).
    RouteResult routeAndCheck( const std::vector<gplan::Waypoint>& waypoints );

    // Like routeAndCheck, but on success also returns the routed geometry
    // (shoved segments + vias) so the host can write it to a board. The geometry
    // is sealed into the PNS session node only — the BOARD is NOT mutated (host
    // applies segs/vias itself, keeping board edits auditable). See .cpp.
    RouteGeom routeAndExtract( const std::vector<gplan::Waypoint>& waypoints );

    // T9 — from a start point on a net, return the nearest still-unconnected
    // ratsnest anchor (the point this net needs to reach) as a Waypoint carrying
    // its PNS layer. nullopt if the net is fully connected or the start is empty.
    // Use this to feed gplan real UNROUTED endpoints instead of guessing pads.
    std::optional<gplan::Waypoint> nearestUnconnected( double x, double y, int pnsLayer );

    // Route AND commit to the PNS world (route-order). Returns PNS's lossless,
    // parent-matched change set: added copper, MODIFY-by-uuid for shoved
    // neighbours (in-place, via links intact), removed-by-uuid. Persists into the
    // world so the next routeAndCommit shoves against this net's copper.
    RouteChange routeAndCommit( const std::vector<gplan::Waypoint>& waypoints );

    // Probe a target point: seedable? foreign copper within `clearance`? nearest
    // foreign distance/net + the same on `otherPnsLayer` (escape hint). Use to
    // flag a congested pad-centre target and re-aim at the drop/escape point.
    TargetProbe probeTarget( double x, double y, int pnsLayer, int net,
                             double clearance, int otherPnsLayer );

    // Shove a whole component (footprint + its connected tracks) so its item at
    // (x,y) moves to (newX,newY), keeping connectivity (PNS COMPONENT_DRAGGER).
    // With allowViolations=false (default) it commits to the world ONLY if the
    // move is clean — i.e. it never endangers connections. Returns the dragged-
    // track change stream (host moves the footprint by (newX-x,newY-y) itself and
    // applies these track mods by UUID). ok=false (nothing committed) if the
    // shove can't be done cleanly.
    RouteChange dragComponent( double x, double y, double newX, double newY,
                               bool allowViolations = false );

    // Long-haul driver: route the waypoints with route_and_commit; if PNS can't
    // reach the target in one shot (a long single leg fragments), insert the
    // farthest-reached point as a CHECKPOINT waypoint and retry — giving PNS
    // stable sub-goals with fresh budgets (AGENT_GUIDE §9). Up to `maxInserts`
    // checkpoints. Returns the committed RouteChange (ok+reached) or, if it still
    // can't reach, an honest ok=false with the final `blocking` point.
    RouteChange routeLongHaul( const std::vector<gplan::Waypoint>& waypoints,
                               int maxInserts = 6 );

    // Speculative component drag: move the component at (x,y) to (newX,newY),
    // evaluate the result (clean? track-length cost?), then DISCARD it (commits
    // nothing). The primitive for a placement search.
    DragProbe probeDrag( double x, double y, double newX, double newY );

    // Router-driven component shoving: probe each candidate position, keep the
    // cleanest/cheapest, and commit that one. `candidates` is a list of {nx,ny}.
    // Returns the chosen position + committed change (or committed=false if no
    // candidate resolves cleanly).
    ShoveResult shoveComponentSearch( double x, double y,
                                      const std::vector<std::vector<double>>& candidates );

    // T-DP — differential pair routing. Click ONE point of either net of the
    // pair (a pad/track); PNS auto-finds its coupled net via
    // DIFF_PAIR_PLACER::FindDpPrimitivePair (matches KiCad's usual "+/-" /
    // "_P/_N" netname convention or explicit pairing) and routes BOTH legs
    // together, offset by DiffPairGap (auto-imported from netclass rules, same
    // as routeAndCommit's ImportSizes). Waypoints are the shared centreline
    // path both legs follow. Commits on success (both legs' geometry in
    // addedSegs/addedVias); same reached/collided honesty as routeAndCommit.
    RouteChange routeDiffPairAndCommit( const std::vector<gplan::Waypoint>& waypoints );

    // T-TUNE — length tuning of an EXISTING routed trace. (x,y) and (endX,endY)
    // must both land on already-placed copper of the same net/trace on
    // pnsLayer (the run being tuned). Replaces the straight run with meanders
    // reaching targetLengthNm (electrical length, nm). Commits on success:
    // change.removedUuids = the original straight segment(s), change.addedSegs
    // = the meander geometry. status/currentLength reflect the actual result
    // even when ok=false (e.g. status=TOO_SHORT if amplitude/space ran out).
    struct TuneResult
    {
        RouteChange change;
        int         status        = -1;   // 0=TOO_SHORT, 1=TOO_LONG, 2=TUNED
        long long   currentLength = 0;
        long long   targetLength  = 0;
    };
    TuneResult tuneLength( double x, double y, double endX, double endY,
                           int pnsLayer, long long targetLengthNm );

    // T-VIA — relocate a single existing via (PNS DM_VIA drag: connectivity-
    // safe, shoves any tracks landing on it as needed — same family as
    // dragComponent/probeDrag but for one via instead of a whole footprint).
    // Use case: a stitching via sitting in a pin's escape corridor — move it
    // out of the way without breaking its GND-plane connection.
    //
    // probeViaMove: speculative — try, evaluate, DISCARD (no commit). The
    // primitive for a placement search (e.g. try several offsets, keep the
    // clean/cheapest one via moveVia).
    DragProbe probeViaMove( double x, double y, double newX, double newY );

    // moveVia: move the via at (x,y) to (newX,newY). Commits only if the move
    // is clean (or allowViolations=true). Returns the change stream (host
    // moves the existing via by uuid to (newX,newY) and applies any track
    // mods by uuid — same lossless contract as dragComponent).
    RouteChange moveVia( double x, double y, double newX, double newY,
                        bool allowViolations = false );

    // Router-driven via shoving: probe each candidate position (probeViaMove),
    // keep the cleanest/cheapest, commit that one (moveVia). Same pattern as
    // shoveComponentSearch, for a single via. `candidates` is a list of {nx,ny}.
    ShoveResult shoveViaSearch( double x, double y,
                               const std::vector<std::vector<double>>& candidates );

    // §4.6 — post-route optimizer pass. Assembles the routed LINE under
    // (x,y,pnsLayer) from the committed PNS world and runs PNS::OPTIMIZER on it
    // (MERGE_SEGMENTS + SMART_PADS: iterative corner-cost reduction + pad-exit
    // rerouting). Commits the improved line to the world through the same
    // lossless parent-matched change stream as routeAndCommit; commits nothing
    // (ok=false) if the optimizer finds no improvement or the result collides.
    OptimizeResult optimizeRoute( double x, double y, int pnsLayer );

    // §4.8 — automated multi-pass mode strategy. Tries RM_Walkaround first (the
    // polite pass: routes around obstacles without shoving existing copper),
    // and only if that fails to reach the target, retries the SAME waypoints
    // once with RM_Shove (the aggressive fallback). The bridge's configured
    // mode (setMode) is restored on every exit path, mirroring the placer-mode
    // restore already used by routeDiffPairAndCommit — a caller's setMode()
    // choice survives a routeWithStrategy() call unchanged.
    RouteChange routeWithStrategy( const std::vector<gplan::Waypoint>& waypoints );

    // T-CORRIDOR — automate the manual cascade-clearing pattern (relocate the
    // nearest blocker, retry, repeat) for a fanout-saturated escape: probe a
    // straight line from (x,y,pnsLayer) toward (x+dirX*radius, y+dirY*radius);
    // if blocked, find the nearest movable board item (via or footprint pad,
    // skipping locked ones) to the blocking point within `radius` of (x,y),
    // shove it outward via shoveViaSearch/shoveComponentSearch (8-direction
    // grid at `stepNm`), and retry the probe. Up to `maxIterations` relocations.
    // Stops as soon as the probe succeeds, a relocation attempt fails to find
    // any clean candidate, or no movable item remains within radius.
    EscapeClearResult clearEscapeCorridor( double x, double y, int pnsLayer,
                                           double dirX, double dirY, double radius,
                                           double stepNm = 100000.0, int maxIterations = 8 );

    BOARD*       board() const { return m_board; }
    PNS::ROUTER* router() const { return m_router.get(); }

private:
    std::unique_ptr<SETTINGS_MANAGER>     m_settings;
    BOARD*                                m_board = nullptr;   // owned via m_boardHolder
    std::shared_ptr<BOARD>                m_boardHolder;
    std::unique_ptr<PNS_KICAD_IFACE_BASE> m_iface;
    std::unique_ptr<PNS::ROUTER>          m_router;
    std::unique_ptr<PNS::ROUTING_SETTINGS> m_routingSettings;
    RouteMode                             m_mode = RouteMode::Shove;   // T12
};

} // namespace gbridge
