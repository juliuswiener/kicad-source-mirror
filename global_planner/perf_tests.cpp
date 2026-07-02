// perf_tests.cpp — T5 (AABB cull) + T-GRID (spatial hash) timing gate.
// Correctness (paths unchanged) is guaranteed by exact culling/hashing (T5's
// AABB check still runs on every grid-query result) and verified by tests.cpp;
// here we only assert the graph build scales acceptably.

#include "planner_core.h"
#include <cstdio>
#include <chrono>
#include <vector>

using namespace gplan;

static Polygon box( double cx, double cy, double s )
{
    double h = s / 2;
    return { { cx - h, cy - h }, { cx + h, cy - h }, { cx + h, cy + h }, { cx - h, cy + h } };
}

int main()
{
    int fail = 0;
    for( int N : { 50, 100, 200, 500, 885 } )
    {
        std::vector<Obstacle> obs;
        for( int i = 0; i < N; ++i )
        {
            double x = ( i % 20 ) * 1.5 + 1, y = ( i / 20 ) * 1.5 + 1;
            obs.push_back( { box( x, y, 0.6 ), ( i % 3 == 0 ), 0 } );
        }
        PlannerParams p; p.trackWidth = 0.15; p.clearance = 0.15; p.kPaths = 5;
        Planner pl( obs, p );

        auto t0 = std::chrono::high_resolution_clock::now();
        int reps = 20; size_t got = 0;
        for( int r = 0; r < reps; ++r )
            got = pl.plan( { 0, 0 }, { ( N % 20 ) * 1.5 + 30.0, ( N / 20 ) * 1.5 } ).size();
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>( t1 - t0 ).count() / reps;

        std::printf( "N=%-4d  %.2f ms/plan  (%zu paths)\n", N, ms, got );
        // Pre-T-GRID: N=200 brute force ~370ms; pre-T5 same ballpark. With T5+
        // T-GRID: N=200 ~28ms, N=885 ~750ms (kubic -> ~quadratic in N; the
        // remaining O(n^2) is the buildEdges() node-PAIR enumeration itself,
        // not the per-edge obstacle scan T-GRID fixed — see ROADMAP.md 2.2).
        // Skip the timing bound under sanitizers (instrumentation dominates).
#if !defined(__SANITIZE_ADDRESS__) && !defined(__SANITIZE_THREAD__)
        if( N == 200 && ms > 200.0 ) { std::printf( "  FAIL: N=200 too slow\n" ); fail++; }
        if( N == 885 && ms > 3000.0 ) { std::printf( "  FAIL: N=885 too slow\n" ); fail++; }
#endif
        if( got == 0 ) { std::printf( "  FAIL: no path\n" ); fail++; }
    }
    // ROADMAP 3.2 -- T-PATTERN speedup: same dense obstacle field (N=885) on
    // ONE Planner instance (corner graph cached after the first plan() call
    // either way), comparing a query that goes THROUGH the field (forces the
    // full corner-graph + Yen's k-shortest search) against one that threads
    // the gap BETWEEN row 0 and row 1 of the same field (pattern fast-path
    // fires; obstacles are close by so edgeWeight's congestion ring-search
    // stays local instead of expanding to scan the whole board, unlike an
    // empty-field corridor far from everything). Reporting only -- not
    // gated, since the absolute speedup is environment-sensitive; the ratio
    // should still be well above 1x.
    {
        const int N = 885;
        std::vector<Obstacle> obs;
        for( int i = 0; i < N; ++i )
        {
            double x = ( i % 20 ) * 1.5 + 1, y = ( i / 20 ) * 1.5 + 1;
            obs.push_back( { box( x, y, 0.6 ), ( i % 3 == 0 ), 0 } );
        }
        PlannerParams p; p.trackWidth = 0.15; p.clearance = 0.15; p.kPaths = 5;
        Planner pl( obs, p );

        int reps = 20;
        auto t0 = std::chrono::high_resolution_clock::now();
        for( int r = 0; r < reps; ++r )
            pl.plan( { 0, 0 }, { ( N % 20 ) * 1.5 + 30.0, ( N / 20 ) * 1.5 } );   // through the field
        auto t1 = std::chrono::high_resolution_clock::now();
        double msBlocked = std::chrono::duration<double, std::milli>( t1 - t0 ).count() / reps;

        auto t2 = std::chrono::high_resolution_clock::now();
        for( int r = 0; r < reps; ++r )
            pl.plan( { 0, 1.75 }, { 30, 1.75 } );   // gap between row 0 and row 1
        auto t3 = std::chrono::high_resolution_clock::now();
        double msPattern = std::chrono::duration<double, std::milli>( t3 - t2 ).count() / reps;

        std::printf( "T-PATTERN N=%-4d  blocked %.2f ms/plan  vs  clear-corridor %.3f ms/plan  (%.1fx)\n",
                     N, msBlocked, msPattern, msBlocked / std::max( msPattern, 1e-6 ) );
    }

    std::printf( fail ? "\nPERF FAIL\n" : "\nPERF OK\n" );
    return fail ? 1 : 0;
}
