// selftest.cpp — builds and exercises the standalone core without Python.
//
// Test 1 (homotopy): one FIXED box blocks the straight line from (0,0) to (10,0).
//   The planner must offer TWO distinct ways around — over the top and under it.
// Test 2 (wiggle room): a MOVABLE track sits in an open gap. It must NOT block the
//   path (only add cost), and a congestion bump must re-rank the candidates.

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
        for( const Point& w : paths[i].waypoints )
            std::printf( " (%.2f,%.2f)", w.x, w.y );
        std::printf( "\n" );
    }
}

int main()
{
    PlannerParams p;
    p.trackWidth = 0.2;
    p.clearance  = 0.2;
    p.kPaths     = 4;

    // --- Test 1: single blocking obstacle, expect 2 distinct homotopies -----
    {
        std::vector<Obstacle> obs = { { box( 5, 0, 3, 3 ), true } };
        Planner planner( obs, p );
        auto paths = planner.plan( { 0, 0 }, { 10, 0 } );
        dump( "Test1 (block in the middle)", paths );

        bool overTop = false, underBottom = false;
        for( const Path& pa : paths )
            for( const Point& w : pa.waypoints )
            {
                if( w.y > 0.5 ) overTop = true;
                if( w.y < -0.5 ) underBottom = true;
            }
        std::printf( "  -> distinct sides found: over-top=%d under-bottom=%d\n\n",
                     overTop, underBottom );
        if( !( overTop && underBottom ) )
            std::printf( "  !! expected both sides\n" );
    }

    // --- Test 2: movable obstacle in an open gap + congestion feedback ------
    {
        std::vector<Obstacle> obs = {
            { box( 5,  1.6, 3, 2 ), true  },   // C
            { box( 5, -1.6, 3, 2 ), true  },   // D
            { box( 5,  0.0, 1, 0.2 ), false }, // movable track in the gap
        };
        Planner planner( obs, p );
        auto a = planner.plan( { 0, 0 }, { 10, 0 } );
        dump( "Test2 (gap with movable track)", a );

        planner.bumpCongestion( { 5, 0 }, 1.5, 5.0 );   // simulate PNS failure in gap
        auto b = planner.plan( { 0, 0 }, { 10, 0 } );
        std::printf( "  after congestion bump: best cost %.2f -> %.2f\n",
                     a.empty() ? -1 : a.front().cost, b.empty() ? -1 : b.front().cost );
    }

    return 0;
}
