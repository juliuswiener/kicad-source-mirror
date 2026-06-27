// selftest.cpp — exercises the standalone core (v2) without Python.
//
// Test 1: single blocking obstacle -> two distinct homotopies (over / under).
// Test 2: movable obstacle in a gap -> doesn't block, congestion bump re-ranks.
// Test 3: layer 0 fully walled off -> planner must via up, cross on layer 1, via down.

#include "planner_core.h"
#include <cstdio>

using namespace gplan;

static Polygon box( double cx, double cy, double w, double h )
{
    double hw = w / 2, hh = h / 2;
    return { { cx - hw, cy - hh }, { cx + hw, cy - hh },
             { cx + hw, cy + hh }, { cx - hw, cy + hh } };
}

static void dump( const char* tag, const std::vector<Path>& paths )
{
    std::printf( "%s: %zu candidate path(s)\n", tag, paths.size() );
    for( size_t i = 0; i < paths.size(); ++i )
    {
        std::printf( "  [%zu] cost=%.2f :", i, paths[i].cost );
        for( const Waypoint& w : paths[i].waypoints )
            std::printf( " (%.2f,%.2f,L%d)", w.p.x, w.p.y, w.layer );
        std::printf( "\n" );
    }
}

int main()
{
    PlannerParams p;
    p.trackWidth = 0.2;
    p.clearance  = 0.2;
    p.kPaths     = 4;

    // --- Test 1: two homotopies around one block --------------------------
    {
        std::vector<Obstacle> obs = { { box( 5, 0, 3, 3 ), true, 0 } };
        Planner planner( obs, p );
        auto paths = planner.plan( { 0, 0 }, { 10, 0 } );
        dump( "Test1 (block in the middle)", paths );
        bool over = false, under = false;
        for( const Path& pa : paths )
            for( const Waypoint& w : pa.waypoints )
            { if( w.p.y > 0.5 ) over = true; if( w.p.y < -0.5 ) under = true; }
        std::printf( "  -> over-top=%d under-bottom=%d %s\n\n", over, under,
                     ( over && under ) ? "OK" : "!! expected both" );
    }

    // --- Test 2: movable in gap + congestion feedback ---------------------
    {
        std::vector<Obstacle> obs = {
            { box( 5,  1.6, 3, 2 ), true,  0 },
            { box( 5, -1.6, 3, 2 ), true,  0 },
            { box( 5,  0.0, 1, 0.2 ), false, 0 },
        };
        Planner planner( obs, p );
        auto a = planner.plan( { 0, 0 }, { 10, 0 } );
        dump( "Test2 (gap with movable track)", a );
        planner.bumpCongestion( { 5, 0 }, 1.5, 5.0 );
        auto b = planner.plan( { 0, 0 }, { 10, 0 } );
        std::printf( "  congestion bump: best %.2f -> %.2f %s\n\n",
                     a.front().cost, b.front().cost,
                     b.front().cost > a.front().cost ? "OK (re-ranked away)" : "!!" );
    }

    // --- Test 3: multi-layer + vias ---------------------------------------
    {
        PlannerParams mp = p;
        mp.layers  = { 0, 1 };   // two copper layers
        mp.viaCost = 3.0;
        // A tall wall on layer 0 blocks every around-route; layer 1 is clear.
        std::vector<Obstacle> obs = { { box( 5, 0, 2, 40 ), true, 0 } };
        Planner planner( obs, mp );
        auto paths = planner.plan( { 0, 0 }, 0, { 10, 0 }, 0 );
        dump( "Test3 (layer 0 walled, must via to layer 1)", paths );
        bool usesL1 = false, viaCount = false;
        for( const Path& pa : paths.empty() ? std::vector<Path>{} : std::vector<Path>{ paths[0] } )
        {
            int changes = 0;
            for( size_t i = 0; i < pa.waypoints.size(); ++i )
            {
                if( pa.waypoints[i].layer == 1 ) usesL1 = true;
                if( i && pa.waypoints[i].layer != pa.waypoints[i - 1].layer ) changes++;
            }
            viaCount = ( changes >= 2 );
        }
        std::printf( "  -> uses layer 1=%d, has via up+down=%d %s\n",
                     usesL1, viaCount,
                     ( usesL1 && viaCount ) ? "OK" : "!! expected layer change" );
    }

    return 0;
}
