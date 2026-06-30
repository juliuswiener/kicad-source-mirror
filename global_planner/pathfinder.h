// pathfinder.h — T11: board-level negotiated-congestion multi-net router.
//
// Header-only, KiCad-free (built on gplan::Planner). Routes a set of nets so the
// outcome is largely independent of net order: each committed net becomes a FIXED
// obstacle for the others (so later nets route around earlier copper), and a
// deadlock (a net with no remaining path) triggers rip-up + a congestion bump on
// a blocking net so it reroutes elsewhere next round. This is the
// McMurchie-Ebeling negotiated-congestion idea at the global-planner layer; the
// detailed shove still happens in PNS via the bridge.

#pragma once

#include "planner_core.h"
#include <cmath>
#include <optional>
#include <vector>

namespace gplan {

struct NetSpec
{
    Point start;  int startLayer = 0;
    Point target; int targetLayer = 0;
};

struct PathFinderResult
{
    std::vector<std::optional<Path>> routes;   // index-aligned with the nets
    bool allRouted = false;
    int  rounds    = 0;
};

namespace detail {

// A routed path's segments become thin FIXED obstacles for the other nets.
inline void appendPathAsObstacles( std::vector<Obstacle>& obs, const Path& p,
                                   double width )
{
    double hw = width / 2.0;
    for( size_t i = 0 ; i + 1 < p.waypoints.size(); ++i )
    {
        if( p.waypoints[i].layer != p.waypoints[i + 1].layer )
            continue;                                   // via, no copper segment
        Point a = p.waypoints[i].p, b = p.waypoints[i + 1].p;
        double dx = b.x - a.x, dy = b.y - a.y;
        double l = std::sqrt( dx * dx + dy * dy );
        if( l < 1e-9 ) l = 1;
        double nx = -dy / l * hw, ny = dx / l * hw;     // segment normal * half-width
        Polygon rect = { { a.x + nx, a.y + ny }, { b.x + nx, b.y + ny },
                         { b.x - nx, b.y - ny }, { a.x - nx, a.y - ny } };
        obs.push_back( { rect, true, p.waypoints[i].layer } );
    }
}

inline Point pathMidpoint( const Path& p )
{
    if( p.waypoints.empty() ) return { 0, 0 };
    const Waypoint& w = p.waypoints[p.waypoints.size() / 2];
    return w.p;
}

} // namespace detail

// Route all nets with negotiated congestion. `base` = fixed board obstacles.
inline PathFinderResult routeNets( std::vector<Obstacle> base, PlannerParams params,
                                   const std::vector<NetSpec>& nets, int maxRounds = 0 )
{
    int N = static_cast<int>( nets.size() );
    if( maxRounds <= 0 ) maxRounds = 4 * N + 4;

    PathFinderResult res;
    res.routes.assign( N, std::nullopt );
    std::vector<CongestionBump> bumps;

    for( res.rounds = 0; res.rounds < maxRounds; ++res.rounds )
    {
        bool progress = false, allDone = true;

        for( int i = 0; i < N; ++i )
        {
            if( res.routes[i] ) continue;
            allDone = false;

            std::vector<Obstacle> obs = base;
            for( int j = 0; j < N; ++j )
                if( j != i && res.routes[j] )
                    detail::appendPathAsObstacles( obs, *res.routes[j], params.trackWidth );

            Planner pl( obs, params );
            for( const CongestionBump& bp : bumps )
                pl.bumpCongestion( bp.center, bp.radius, bp.factor );

            auto paths = pl.plan( nets[i].start, nets[i].startLayer,
                                  nets[i].target, nets[i].targetLayer );
            if( !paths.empty() )
            {
                res.routes[i] = paths.front();
                progress = true;
            }
        }

        if( allDone ) { res.allRouted = true; break; }

        if( !progress )
        {
            // Deadlock: rip up one committed net and penalize its corridor so it
            // takes a different route next round, freeing space for the blocked net.
            for( int j = 0; j < N; ++j )
                if( res.routes[j] )
                {
                    bumps.push_back( { detail::pathMidpoint( *res.routes[j] ),
                                       params.clearance * 10.0, 5.0 } );
                    res.routes[j].reset();
                    break;
                }
        }
    }

    res.allRouted = true;
    for( const auto& r : res.routes ) if( !r ) res.allRouted = false;
    return res;
}

} // namespace gplan
