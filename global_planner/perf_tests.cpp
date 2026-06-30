// perf_tests.cpp — T5 spatial-cull timing gate.
// Correctness (paths unchanged) is guaranteed by exact AABB pruning and verified
// by tests.cpp; here we only assert the graph build scales acceptably.

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
    for( int N : { 50, 100, 200 } )
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
        for( int r = 0; r < reps; ++r ) got = pl.plan( { 0, 0 }, { 31, 16 } ).size();
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>( t1 - t0 ).count() / reps;

        std::printf( "N=%-4d  %.2f ms/plan  (%zu paths)\n", N, ms, got );
        // Pre-cull brute force was ~370 ms at N=200; pruned must be well under.
        // Skip the timing bound under sanitizers (instrumentation dominates).
#if !defined(__SANITIZE_ADDRESS__) && !defined(__SANITIZE_THREAD__)
        if( N == 200 && ms > 200.0 ) { std::printf( "  FAIL: N=200 too slow\n" ); fail++; }
#endif
        if( got == 0 ) { std::printf( "  FAIL: no path\n" ); fail++; }
    }
    std::printf( fail ? "\nPERF FAIL\n" : "\nPERF OK\n" );
    return fail ? 1 : 0;
}
