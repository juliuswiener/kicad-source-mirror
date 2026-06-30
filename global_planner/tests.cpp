// tests.cpp — assertion-based test suite for the standalone core.
// Build with sanitizers; non-zero exit on any failure (used by ctest).

#include "planner_core.h"
#include <cmath>
#include <cstdio>
#include <thread>
#include <atomic>
#include <vector>

using namespace gplan;

// point-in-polygon (test-local copy) for the T2 via-site invariant.
static bool ptInPoly( Point p, const Polygon& poly )
{
    bool in = false; size_t n = poly.size();
    for( size_t i = 0, j = n - 1; i < n; j = i++ )
        if( ( ( poly[i].y > p.y ) != ( poly[j].y > p.y ) ) &&
            ( p.x < ( poly[j].x - poly[i].x ) * ( p.y - poly[i].y )
                        / ( poly[j].y - poly[i].y ) + poly[i].x ) )
            in = !in;
    return in;
}

static int g_pass = 0, g_fail = 0;
#define CHECK( cond, msg ) do { \
    if( cond ) g_pass++; else { g_fail++; std::printf( "FAIL: %s\n", msg ); } } while( 0 )

static Polygon box( double cx, double cy, double w, double h )
{
    double hw = w / 2, hh = h / 2;
    return { { cx - hw, cy - hh }, { cx + hw, cy - hh },
             { cx + hw, cy + hh }, { cx - hw, cy + hh } };
}

// --- tiny geometry for the clearance assertion ----------------------------
static double dps( Point p, Point a, Point b )
{
    double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
    if( L2 < 1e-12 ) return std::hypot( p.x - a.x, p.y - a.y );
    double t = ( ( p.x - a.x ) * dx + ( p.y - a.y ) * dy ) / L2;
    t = std::max( 0.0, std::min( 1.0, t ) );
    return std::hypot( p.x - ( a.x + dx * t ), p.y - ( a.y + dy * t ) );
}
static double segPolyDist( Point a, Point b, const Polygon& poly )
{
    double best = 1e18;
    size_t n = poly.size();
    for( size_t i = 0, j = n - 1; i < n; j = i++ )
    {
        // sample both segments against each other crudely
        for( int k = 0; k <= 8; ++k )
        {
            double t = k / 8.0;
            Point s = { a.x + ( b.x - a.x ) * t, a.y + ( b.y - a.y ) * t };
            best = std::min( best, dps( s, poly[j], poly[i] ) );
        }
    }
    return best;
}

// Assert a path keeps `margin` from every fixed obstacle on its layer.
static bool pathClears( const Path& p, const std::vector<Obstacle>& obs, double margin )
{
    for( size_t i = 0; i + 1 < p.waypoints.size(); ++i )
    {
        if( p.waypoints[i].layer != p.waypoints[i + 1].layer )
            continue; // via, not a track segment
        for( const Obstacle& o : obs )
        {
            if( !o.fixed || o.layer != p.waypoints[i].layer )
                continue;
            // skip the obstacle the endpoint sits on (source/target pad)
            double da = segPolyDist( p.waypoints[i].p, p.waypoints[i].p, o.poly );
            double db = segPolyDist( p.waypoints[i + 1].p, p.waypoints[i + 1].p, o.poly );
            if( da < 1e-6 || db < 1e-6 )
                continue;
            if( segPolyDist( p.waypoints[i].p, p.waypoints[i + 1].p, o.poly ) < margin * 0.85 )
                return false;
        }
    }
    return true;
}

int main()
{
    PlannerParams base;
    base.trackWidth = 0.2;
    base.clearance  = 0.2;
    base.kPaths     = 5;
    const double margin = base.clearance + base.trackWidth / 2.0;

    // 1. Open field: straight path exists.
    {
        Planner pl( {}, base );
        auto paths = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( !paths.empty(), "open field reachable" );
        CHECK( paths.front().waypoints.size() == 2, "open field is a straight line" );
    }

    // 2. start == target.
    {
        Planner pl( {}, base );
        auto paths = pl.plan( { 3, 3 }, { 3, 3 } );
        CHECK( !paths.empty(), "degenerate start==target returns a path" );
    }

    // 3. Two homotopies around a single block, both clearing it.
    {
        std::vector<Obstacle> obs = { { box( 5, 0, 3, 3 ), true, 0 } };
        Planner pl( obs, base );
        auto paths = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( paths.size() >= 2, "block: at least two homotopies" );
        bool over = false, under = false, clears = true;
        for( const Path& p : paths )
        {
            for( const Waypoint& w : p.waypoints )
            { if( w.p.y > 0.4 ) over = true; if( w.p.y < -0.4 ) under = true; }
            clears = clears && pathClears( p, obs, margin );
        }
        CHECK( over && under, "block: both sides offered" );
        CHECK( clears, "block: every path clears the obstacle by margin" );
    }

    // 4. Movable obstacle does NOT block; congestion bump re-ranks.
    {
        std::vector<Obstacle> obs = {
            { box( 5,  1.6, 3, 2 ), true,  0 },
            { box( 5, -1.6, 3, 2 ), true,  0 },
            { box( 5,  0.0, 1, 0.2 ), false, 0 },
        };
        Planner pl( obs, base );
        auto a = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( !a.empty() && a.front().waypoints.size() == 2,
               "movable does not block the straight gap" );
        pl.bumpCongestion( { 5, 0 }, 1.5, 5.0 );
        auto b = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( !b.empty() && b.front().cost > a.front().cost,
               "congestion bump re-ranks away from the gap" );
    }

    // 5. Route FROM a pad: start sits inside its own fixed pad hull.
    {
        std::vector<Obstacle> obs = {
            { box( 0, 0, 1, 1 ), true, 0 },   // source pad at start
            { box( 10, 0, 1, 1 ), true, 0 },  // target pad
        };
        Planner pl( obs, base );
        auto paths = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( !paths.empty(), "can route starting from inside a pad hull" );
    }

    // 6. Fully boxed-in start -> unreachable -> empty (no crash).
    {
        // Walls 1.0 thick (> 2*margin) and heavily overlapping at the corners,
        // so there is no diagonal escape even after margin inflation.
        std::vector<Obstacle> obs = {
            { box( 0,  2, 8, 1 ), true, 0 }, { box( 0, -2, 8, 1 ), true, 0 },
            { box( 4,  0, 1, 8 ), true, 0 }, { box( -4, 0, 1, 8 ), true, 0 },
        };
        Planner pl( obs, base );
        auto paths = pl.plan( { 0, 0 }, { 20, 0 } );
        CHECK( paths.empty(), "fully enclosed start is unreachable" );
    }

    // 7. Non-convex fixed polygon is handled defensively (no crash, still clears).
    {
        Polygon L = { {3,-1},{7,-1},{7,1},{6,1},{6,0},{3,0} }; // concave-ish
        std::vector<Obstacle> obs = { { L, true, 0 } };
        Planner pl( obs, base );
        auto paths = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( !paths.empty(), "non-convex fixed input handled" );
    }

    // 8. Multi-layer: walled on layer 0 -> must via to layer 1 and back.
    {
        PlannerParams mp = base;
        mp.layers = { 0, 1 };
        mp.viaCost = 3.0;
        std::vector<Obstacle> obs = { { box( 5, 0, 2, 40 ), true, 0 } };
        Planner pl( obs, mp );
        auto paths = pl.plan( { 0, 0 }, 0, { 10, 0 }, 0 );
        CHECK( !paths.empty(), "multilayer: path exists via layer 1" );
        int changes = 0; bool usesL1 = false;
        for( size_t i = 0; i < paths.front().waypoints.size(); ++i )
        {
            if( paths.front().waypoints[i].layer == 1 ) usesL1 = true;
            if( i && paths.front().waypoints[i].layer != paths.front().waypoints[i-1].layer )
                changes++;
        }
        CHECK( usesL1 && changes >= 2, "multilayer: vias up and down" );
    }

    // 9. Distinct-path dedup: open field returns exactly one path.
    {
        Planner pl( {}, base );
        auto paths = pl.plan( { 0, 0 }, { 5, 0 } );
        CHECK( paths.size() == 1, "no spurious duplicate paths in open field" );
    }

    // T3. Bad params are clamped, not fatal.
    {
        PlannerParams bad = base;
        bad.kPaths = 0; bad.viaCost = -5; bad.clearance = -1; bad.reusePenalty = 0.1;
        Planner pl( { { box( 5, 0, 3, 3 ), true, 0 } }, bad );
        auto paths = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( !paths.empty(), "T3: bad params clamped, still routes" );
    }

    // T2. A via never lands inside a fixed hull (interior via-site rejection).
    {
        PlannerParams mp = base;
        mp.layers = { 0, 1 };
        mp.viaCost = 3.0;
        std::vector<Obstacle> obs = {
            { box( 5, 0, 2, 40 ), true, 0 },   // wall on L0 forces a via
            { box( 0, 0, 1, 1 ), true, 1 },    // fixed pad on L1 over the (0,0) via column
        };
        Planner pl( obs, mp );
        auto paths = pl.plan( { 0, 0 }, 0, { 10, 0 }, 0 );
        bool viaInsideFixed = false;
        for( const Path& p : paths )
            for( size_t i = 1; i < p.waypoints.size(); ++i )
                if( p.waypoints[i].layer != p.waypoints[i-1].layer )           // a via
                    for( const Obstacle& o : obs )
                        if( o.fixed && o.layer == p.waypoints[i].layer
                            && ptInPoly( p.waypoints[i].p, o.poly ) )
                            viaInsideFixed = true;
        CHECK( !viaInsideFixed, "T2: no via lands inside a fixed hull" );
    }

    // T1. Concurrent plan()+bumpCongestion on ONE Planner: no crash, all valid.
    {
        Planner pl( { { box( 5, 0, 3, 3 ), true, 0 } }, base );
        std::atomic<int> ok{ 0 }, bad{ 0 };
        std::vector<std::thread> ts;
        for( int t = 0; t < 8; ++t )
            ts.emplace_back( [&]{
                for( int i = 0; i < 50; ++i )
                {
                    auto paths = pl.plan( { 0, 0 }, { 10, 0 } );
                    ( paths.empty() ? bad : ok )++;
                    pl.bumpCongestion( { 5.0, 0.0 }, 1.0, 2.0 );
                }
            } );
        for( auto& th : ts ) th.join();
        pl.clearCongestion();
        CHECK( bad == 0 && ok == 8 * 50, "T1: concurrent plan/bump safe + valid" );
    }

    // T6. Exact edge blocking catches a thin wall placed between old sample points.
    {
        // 30-long edge, old K=24 sampling spacing 1.25; wall at x=15.6 (off-grid)
        // inflated ~0.3 wide → no old sample landed inside, but it truly blocks.
        std::vector<Obstacle> obs = { { box( 15.6, 0, 0.1, 40 ), true, 0 } };
        Planner pl( obs, base );
        auto paths = pl.plan( { 0, 0 }, { 30, 0 } );
        bool straight = false;
        for( const Path& p : paths )
            if( p.waypoints.size() == 2 ) straight = true;   // direct line = crosses wall
        CHECK( !straight, "T6: thin wall blocks the direct edge (exact, no sampling miss)" );
    }

    std::printf( "\n%d passed, %d failed\n", g_pass, g_fail );
    return g_fail ? 1 : 0;
}
