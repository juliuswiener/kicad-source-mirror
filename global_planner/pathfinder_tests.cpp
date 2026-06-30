// pathfinder_tests.cpp — T11 multi-net negotiated-congestion gate.

#include "pathfinder.h"
#include <cstdio>

using namespace gplan;

static int g_pass = 0, g_fail = 0;
#define CHECK( cond, msg ) do { \
    if( cond ) g_pass++; else { g_fail++; std::printf( "FAIL: %s\n", msg ); } } while( 0 )

static Polygon box( double cx, double cy, double w, double h )
{
    double hw = w / 2, hh = h / 2;
    return { { cx - hw, cy - hh }, { cx + hw, cy - hh },
             { cx + hw, cy + hh }, { cx - hw, cy + hh } };
}

static bool detoured( const std::optional<Path>& p )
{
    if( !p ) return false;
    for( const Waypoint& w : p->waypoints )
        if( std::fabs( w.p.y ) > 1.0 ) return true;   // went around the walls
    return false;
}

int main()
{
    PlannerParams params;
    params.trackWidth = 0.2;
    params.clearance  = 0.2;
    params.kPaths     = 6;

    // One narrow channel (~1 track) between two walls, with a detour over the top.
    // Endpoints are kept clear of each other's inflated copper (±~0.3) so a net is
    // genuinely blocked by another's copper, not waved through by pad-emergence.
    std::vector<Obstacle> base = {
        { box( 5,  1.0, 6, 1.2 ), true, 0 },   // top wall    x[2,8] y[0.4,1.6]
        { box( 5, -1.0, 6, 1.2 ), true, 0 },   // bottom wall x[2,8] y[-1.6,-0.4]
    };

    // Two nets: A takes the channel; B (entering at y=0.6) is then blocked and
    // must detour over the top.
    std::vector<NetSpec> nets = {
        { { 0,  0.0 }, 0, { 10,  0.0 }, 0 },
        { { 0,  0.6 }, 0, { 10,  0.6 }, 0 },
    };

    auto res = routeNets( base, params, nets );
    CHECK( res.allRouted, "T11: both nets routed" );
    int detours = detoured( res.routes[0] ) + detoured( res.routes[1] );
    CHECK( detours >= 1, "T11: contended gap forces one net to detour" );

    // Order independence: reversing the net order still routes both.
    std::vector<NetSpec> rev = { nets[1], nets[0] };
    auto res2 = routeNets( base, params, rev );
    CHECK( res2.allRouted, "T11: both nets routed regardless of order" );

    // Three nets, one channel: still all routed (two detour), bounded rounds.
    std::vector<NetSpec> three = {
        { { 0,  0.0 }, 0, { 10,  0.0 }, 0 },
        { { 0,  0.6 }, 0, { 10,  0.6 }, 0 },
        { { 0, -0.6 }, 0, { 10, -0.6 }, 0 },
    };
    auto res3 = routeNets( base, params, three );
    CHECK( res3.allRouted, "T11: 3 nets through 1 gap all routed (2 detour)" );

    std::printf( "\n%d passed, %d failed\n", g_pass, g_fail );
    return g_fail ? 1 : 0;
}
