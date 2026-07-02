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

// Exact path-set equality (cost + waypoints + layers), for the 2.5 cache test.
static bool samePaths( const std::vector<Path>& a, const std::vector<Path>& b )
{
    if( a.size() != b.size() )
        return false;
    for( size_t i = 0; i < a.size(); ++i )
    {
        if( std::fabs( a[i].cost - b[i].cost ) > 1e-9 )
            return false;
        if( a[i].waypoints.size() != b[i].waypoints.size() )
            return false;
        for( size_t j = 0; j < a[i].waypoints.size(); ++j )
        {
            if( a[i].waypoints[j].layer != b[i].waypoints[j].layer )
                return false;
            if( std::fabs( a[i].waypoints[j].p.x - b[i].waypoints[j].p.x ) > 1e-9
                || std::fabs( a[i].waypoints[j].p.y - b[i].waypoints[j].p.y ) > 1e-9 )
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

    // 2.5 — graph cache + region culling: repeated plan() reuses the cached
    // corner graph (counter proves it), cached results are identical to a cold
    // (uncached) instance, and bumps/region changes invalidate correctly.
    {
        std::vector<Obstacle> obs = {
            { box( 5,  1.6, 3, 2 ), true, 0 },
            { box( 5, -1.6, 3, 2 ), true, 0 },
            { box( 30, 0, 3, 3 ), true, 0 },      // far obstacle (outside region below)
        };
        Planner pl( obs, base );
        CHECK( pl.graphBuildCount() == 0, "2.5: no graph built before first plan()" );

        auto a = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( pl.graphBuildCount() == 1, "2.5: first plan() builds the graph" );

        auto b = pl.plan( { 0, 0 }, { 10, 0 } );
        auto c = pl.plan( { 0, 3 }, { 10, 3 } );  // different query, same obstacles
        CHECK( pl.graphBuildCount() == 1, "2.5: repeated plan() reuses the cached graph" );
        CHECK( !c.empty(), "2.5: cached graph serves a different start/target" );
        CHECK( samePaths( a, b ), "2.5: cache-hit result identical to cache-miss result" );

        Planner fresh( obs, base );               // cold instance = uncached path
        CHECK( samePaths( b, fresh.plan( { 0, 0 }, { 10, 0 } ) ),
               "2.5: cached results match an uncached instance" );

        pl.bumpCongestion( { 5, 0 }, 1.5, 5.0 );
        auto d = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( pl.graphBuildCount() == 2, "2.5: bumpCongestion invalidates the cache" );
        CHECK( !d.empty() && d.front().cost > a.front().cost,
               "2.5: post-bump rebuild sees the new congestion" );
        pl.clearCongestion();
        auto e = pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( pl.graphBuildCount() == 3, "2.5: clearCongestion invalidates the cache" );
        CHECK( samePaths( a, e ), "2.5: after clearCongestion results are back to baseline" );

        BBox reg{ -2, -6, 14, 6 };                // excludes the far obstacle at x=30
        auto r1 = pl.plan( { 0, 0 }, { 10, 0 }, reg );
        CHECK( pl.graphBuildCount() == 4, "2.5: new region rebuilds the graph once" );
        auto r2 = pl.plan( { 0, 0 }, { 10, 0 }, reg );
        CHECK( pl.graphBuildCount() == 4, "2.5: same region reuses the cached graph" );
        CHECK( !r1.empty() && samePaths( r1, r2 ), "2.5: region plan cached == uncached" );

        pl.clearRegion();
        (void) pl.plan( { 0, 0 }, { 10, 0 } );
        CHECK( pl.graphBuildCount() == 5, "2.5: clearRegion invalidates the cache" );
    }

    // T-MULTI (ROADMAP 4.5): multi-terminal / bus routing. The greedy
    // Steiner-tree connects terminals by actual path cost, not input order --
    // permuting the terminal list must not change the total tree cost (only
    // which paths are returned), unlike a naive start->t1->t2->... chain.
    {
        std::vector<Obstacle> obs;   // open field: isolates ordering effects from routing detail
        Planner pl( obs, base );

        Waypoint A{ { 0, 0 }, 0 }, B{ { 1, 0 }, 0 }, C{ { 100, 0 }, 0 }, D{ { 2, 0 }, 0 };

        auto sumCost = []( const std::vector<Path>& tree )
        {
            double s = 0.0;
            for( const Path& p : tree ) s += p.cost;
            return s;
        };

        auto tree1 = pl.planMultiTerminal( { A, C, B, D } );   // deliberately bad input order
        auto tree2 = pl.planMultiTerminal( { B, D, A, C } );   // different start + order

        CHECK( tree1.size() == 3, "T-MULTI: n-1 paths for 4 terminals" );
        CHECK( tree2.size() == 3, "T-MULTI: n-1 paths regardless of input order" );
        for( const Path& p : tree1 )
            CHECK( !p.waypoints.empty(), "T-MULTI: every connection is a real path" );

        CHECK( std::fabs( sumCost( tree1 ) - sumCost( tree2 ) ) < 1e-6,
               "T-MULTI: total tree cost is order-independent (greedy picks by cost, not input order)" );

        CHECK( pl.planMultiTerminal( { A } ).empty(),
               "T-MULTI: fewer than 2 terminals returns no paths" );
    }

    // T-PATTERN (ROADMAP 3.2): direct/L-shape fast-first-pass tried before
    // Yen's k-shortest search.
    {
        // A: direct diagonal blocked, both L-corners clear -> pattern still
        // fires, returning a 3-waypoint L path (not a full aStar route).
        std::vector<Obstacle> obs = { { box( 5, 5, 4, 4 ), true, 0 } };
        Planner pl( obs, base );
        auto paths = pl.plan( { 0, 0 }, { 10, 10 } );
        CHECK( !paths.empty(), "T-PATTERN: L-shape around a diagonal block" );
        CHECK( paths.front().waypoints.size() == 3,
               "T-PATTERN: pattern L-shape is a single corner, not a full aStar route" );
        CHECK( pathClears( paths.front(), obs, margin ),
               "T-PATTERN: pattern L-shape clears the obstacle by margin" );
    }
    {
        // B: direct AND both L-corners blocked -> pattern must decline and
        // fall through to the full corner-graph search, which still finds a
        // route around every obstacle.
        std::vector<Obstacle> obs = {
            { box( 5, 5, 3, 3 ), true, 0 },     // blocks the direct diagonal
            { box( 8, 0, 3, 2 ), true, 0 },     // blocks leg (0,0)->(10,0)
            { box( 0, 8, 2, 3 ), true, 0 },     // blocks leg (0,0)->(0,10)
        };
        Planner pl( obs, base );
        auto paths = pl.plan( { 0, 0 }, { 10, 10 } );
        CHECK( !paths.empty(), "T-PATTERN: fallback to full search when every shape is blocked" );
        bool anyClears = false;
        for( const Path& p : paths )
            if( pathClears( p, obs, margin ) ) anyClears = true;
        CHECK( anyClears, "T-PATTERN: fallback route clears every obstacle by margin" );
    }

    // T-TAUT (ROADMAP 3.4): taut-string post-step. A weaving obstacle course
    // forces the full corner-graph search (direct line blocked, both pattern
    // L-corners degenerate since start/target are collinear); the taut-string
    // pass then keeps the returned path close to the straight-line length
    // instead of hugging every intermediate hull corner the graph search
    // happened to visit.
    {
        std::vector<Obstacle> obs = {
            { box( 4, 1, 2, 2 ), true, 0 },
            { box( 8, -1, 2, 2 ), true, 0 },
            { box( 12, 1, 2, 2 ), true, 0 },
            { box( 16, -1, 2, 2 ), true, 0 },
        };
        Planner pl( obs, base );
        auto paths = pl.plan( { 0, 0 }, { 20, 0 } );
        CHECK( !paths.empty(), "T-TAUT: weaving obstacle course still routes" );
        const Path& p = paths.front();
        CHECK( pathClears( p, obs, margin ), "T-TAUT: taut path still clears every obstacle" );

        double len = 0.0;
        for( size_t i = 0; i + 1 < p.waypoints.size(); ++i )
            if( p.waypoints[i].layer == p.waypoints[i + 1].layer )
                len += std::hypot( p.waypoints[i + 1].p.x - p.waypoints[i].p.x,
                                    p.waypoints[i + 1].p.y - p.waypoints[i].p.y );
        double straight = 20.0;
        std::printf( "T-TAUT: %zu waypoints, length %.3f (straight-line %.3f, ratio %.3f)\n",
                     p.waypoints.size(), len, straight, len / straight );
        CHECK( len < straight * 1.5,
               "T-TAUT: taut path length stays close to the straight-line lower bound" );
    }

    std::printf( "\n%d passed, %d failed\n", g_pass, g_fail );
    return g_fail ? 1 : 0;
}
