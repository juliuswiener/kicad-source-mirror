// planner_core.cpp  — see planner_core.h

#include "planner_core.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace gplan {

// ---------------------------------------------------------------------------
// Small 2D geometry helpers (self-contained, no external deps)
// ---------------------------------------------------------------------------
namespace {

inline double dot( Point a, Point b )  { return a.x * b.x + a.y * b.y; }
inline double cross( Point a, Point b ){ return a.x * b.y - a.y * b.x; }
inline Point  sub( Point a, Point b )  { return { a.x - b.x, a.y - b.y }; }
inline double len( Point a )           { return std::sqrt( a.x * a.x + a.y * a.y ); }
inline double dist( Point a, Point b ) { return len( sub( a, b ) ); }

// Distance from point p to segment [a,b].
double distPointSeg( Point p, Point a, Point b )
{
    Point ab = sub( b, a );
    double L2 = dot( ab, ab );
    if( L2 < 1e-12 )
        return dist( p, a );
    double t = dot( sub( p, a ), ab ) / L2;
    t = std::clamp( t, 0.0, 1.0 );
    return dist( p, { a.x + ab.x * t, a.y + ab.y * t } );
}

bool segsIntersect( Point a, Point b, Point c, Point d )
{
    auto sgn = []( double v ) { return ( v > 0 ) - ( v < 0 ); };
    double d1 = cross( sub( d, c ), sub( a, c ) );
    double d2 = cross( sub( d, c ), sub( b, c ) );
    double d3 = cross( sub( b, a ), sub( c, a ) );
    double d4 = cross( sub( b, a ), sub( d, a ) );
    if( sgn( d1 ) != sgn( d2 ) && sgn( d3 ) != sgn( d4 ) )
        return true;
    return false; // collinear-overlap treated as non-crossing; good enough for v1
}

// Minimum distance between two segments (0 if they intersect).
double segSegDist( Point a, Point b, Point c, Point d )
{
    if( segsIntersect( a, b, c, d ) )
        return 0.0;
    return std::min( { distPointSeg( a, c, d ), distPointSeg( b, c, d ),
                       distPointSeg( c, a, b ), distPointSeg( d, a, b ) } );
}

bool pointInPolygon( Point p, const Polygon& poly )
{
    bool in = false;
    size_t n = poly.size();
    for( size_t i = 0, j = n - 1; i < n; j = i++ )
    {
        const Point& vi = poly[i];
        const Point& vj = poly[j];
        if( ( ( vi.y > p.y ) != ( vj.y > p.y ) ) &&
            ( p.x < ( vj.x - vi.x ) * ( p.y - vi.y ) / ( vj.y - vi.y ) + vi.x ) )
            in = !in;
    }
    return in;
}

// Minimum distance from segment [a,b] to polygon (0 if it touches/enters).
double distSegPolygon( Point a, Point b, const Polygon& poly )
{
    if( pointInPolygon( a, poly ) || pointInPolygon( b, poly ) )
        return 0.0;
    double best = std::numeric_limits<double>::max();
    size_t n = poly.size();
    for( size_t i = 0, j = n - 1; i < n; j = i++ )
    {
        double d = segSegDist( a, b, poly[j], poly[i] );
        if( d == 0.0 )
            return 0.0;
        best = std::min( best, d );
    }
    return best;
}

double signedArea( const Polygon& poly )
{
    double a = 0;
    size_t n = poly.size();
    for( size_t i = 0, j = n - 1; i < n; j = i++ )
        a += cross( poly[j], poly[i] );
    return a / 2.0;
}

// Ensure counter-clockwise winding (interior on the left of each edge).
Polygon asCCW( Polygon poly )
{
    if( signedArea( poly ) < 0 )
        std::reverse( poly.begin(), poly.end() );
    return poly;
}

// Intersection of two lines, each given by a point and a direction.
Point lineIntersect( Point p1, Point d1, Point p2, Point d2 )
{
    double den = cross( d1, d2 );
    if( std::fabs( den ) < 1e-12 )
        return p1; // parallel: degenerate, fall back
    double t = cross( sub( p2, p1 ), d2 ) / den;
    return { p1.x + d1.x * t, p1.y + d1.y * t };
}

// Offset a convex CCW polygon outward by distance d (Minkowski with a disk,
// corners kept sharp). New vertex = intersection of the two adjacent offset edges.
Polygon offsetConvex( const Polygon& ccw, double d )
{
    size_t n = ccw.size();
    if( n < 3 )
        return ccw;

    std::vector<Point> offA( n ), offDir( n ); // offset edge i: point + direction
    for( size_t i = 0; i < n; ++i )
    {
        Point a = ccw[i], b = ccw[( i + 1 ) % n];
        Point dir = sub( b, a );
        double l = len( dir );
        if( l < 1e-9 ) l = 1;
        dir = { dir.x / l, dir.y / l };
        Point outward = { dir.y, -dir.x };       // right of CCW edge = outward
        offA[i]   = { a.x + outward.x * d, a.y + outward.y * d };
        offDir[i] = dir;
    }

    Polygon out( n );
    for( size_t i = 0; i < n; ++i )
    {
        size_t prev = ( i + n - 1 ) % n;        // edges meeting at vertex i
        out[i] = lineIntersect( offA[prev], offDir[prev], offA[i], offDir[i] );
    }
    return out;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Planner
// ---------------------------------------------------------------------------

Planner::Planner( std::vector<Obstacle> obstacles, PlannerParams params )
    : m_obstacles( std::move( obstacles ) ), m_params( params )
{
    const double margin = m_params.clearance + m_params.trackWidth / 2.0;
    const double eps    = margin * 0.02;

    for( const Obstacle& ob : m_obstacles )
    {
        if( ob.fixed )
        {
            Polygon ccw = asCCW( ob.poly );
            m_fixedOrig.push_back( ccw );
            m_fixedInflated.push_back( offsetConvex( ccw, margin ) );
            m_fixedBlock.push_back( offsetConvex( ccw, margin - eps ) );
        }
        else
        {
            m_movable.push_back( ob.poly );
        }
    }
}

void Planner::bumpCongestion( Point where, double radius, double factor )
{
    m_bumps.push_back( { where, radius, factor } );
}

void Planner::clearCongestion()
{
    m_bumps.clear();
}

void Planner::buildNodes( const Point& start, const Point& target )
{
    m_nodes.clear();
    m_nodes.push_back( start );    // index 0
    m_nodes.push_back( target );   // index 1

    // Graph nodes = corners of the margin-inflated FIXED hulls. A path routed
    // through these corners is guaranteed to clear the real obstacle by `margin`.
    for( const Polygon& hull : m_fixedInflated )
        for( const Point& v : hull )
            m_nodes.push_back( v );
}

bool Planner::edgeBlocked( const Point& a, const Point& b ) const
{
    // Blocked iff the segment strictly penetrates an inflated hull. Grazing along
    // a hull boundary (the usual case for a tangent edge) is allowed.
    const int K = 24;
    for( const Polygon& block : m_fixedBlock )
    {
        for( int k = 1; k < K; ++k )
        {
            double t = static_cast<double>( k ) / K;
            Point  s = { a.x + ( b.x - a.x ) * t, a.y + ( b.y - a.y ) * t };
            if( pointInPolygon( s, block ) )
                return true;
        }
    }
    return false;
}

double Planner::edgeWeight( const Point& a, const Point& b ) const
{
    const double margin = m_params.clearance + m_params.trackWidth / 2.0;
    const double pitch  = m_params.trackWidth + m_params.clearance;
    const double base   = dist( a, b );

    // Distance to the nearest FIXED obstacle along this corridor.
    double dFix = std::numeric_limits<double>::max();
    for( const Polygon& hull : m_fixedOrig )
        dFix = std::min( dFix, distSegPolygon( a, b, hull ) );
    if( dFix == std::numeric_limits<double>::max() )
        dFix = 10.0 * pitch; // open field: plenty of room

    // Channel capacity ~ how many tracks fit in the gap this corridor squeezes through.
    double gap      = 2.0 * dFix;                  // approx: room on both sides
    double capacity = std::max( 1.0, std::floor( gap / pitch ) );

    // Usage = movable copper this corridor would have to share / shove.
    double usage = 0.0;
    for( const Polygon& mv : m_movable )
        if( distSegPolygon( a, b, mv ) < margin )
            usage += 1.0;

    // PathFinder feedback: areas where PNS already failed cost more.
    for( const CongestionBump& bp : m_bumps )
        if( distPointSeg( bp.center, a, b ) < bp.radius )
            usage += bp.factor;

    double load       = usage / capacity;
    double congestion = m_params.wCongestion * load * load * base;
    double tightness  = m_params.wTightness * ( margin / std::max( dFix, margin ) ) * base;

    return base + congestion + tightness;
}

void Planner::buildEdges()
{
    int n = static_cast<int>( m_nodes.size() );
    m_adj.assign( n, {} );
    m_edgeList.clear();

    for( int i = 0; i < n; ++i )
    {
        for( int j = i + 1; j < n; ++j )
        {
            if( edgeBlocked( m_nodes[i], m_nodes[j] ) )
                continue;
            double w = edgeWeight( m_nodes[i], m_nodes[j] );
            int id = static_cast<int>( m_edgeList.size() );
            m_edgeList.push_back( { i, j, w } );
            m_adj[i].push_back( { j, id } );
            m_adj[j].push_back( { i, id } );
        }
    }
}

std::vector<int> Planner::aStar( int src, int dst,
                                 const std::vector<double>& edgeMul ) const
{
    int n = static_cast<int>( m_nodes.size() );
    std::vector<double> g( n, std::numeric_limits<double>::max() );
    std::vector<int>    prev( n, -1 );

    auto h = [&]( int i ) { return dist( m_nodes[i], m_nodes[dst] ); };

    using QE = std::pair<double, int>; // (f = g + h, node)
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;

    g[src] = 0.0;
    pq.push( { h( src ), src } );

    while( !pq.empty() )
    {
        auto [f, u] = pq.top();
        pq.pop();
        if( u == dst )
            break;
        if( f - h( u ) > g[u] + 1e-9 )
            continue; // stale

        for( const Edge& e : m_adj[u] )
        {
            double w = m_edgeList[e.id].weight * edgeMul[e.id];
            double ng = g[u] + w;
            if( ng + 1e-9 < g[e.to] )
            {
                g[e.to] = ng;
                prev[e.to] = u;
                pq.push( { ng + h( e.to ), e.to } );
            }
        }
    }

    std::vector<int> path;
    if( prev[dst] == -1 && src != dst )
        return path; // unreachable
    for( int u = dst; u != -1; u = prev[u] )
        path.push_back( u );
    std::reverse( path.begin(), path.end() );
    return path;
}

namespace {

// Drop interior points that lie (nearly) on the line between their neighbours.
std::vector<Point> simplify( const std::vector<Point>& pts )
{
    if( pts.size() <= 2 )
        return pts;
    std::vector<Point> out;
    out.push_back( pts.front() );
    for( size_t i = 1; i + 1 < pts.size(); ++i )
        if( distPointSeg( pts[i], out.back(), pts[i + 1] ) > 1e-6 )
            out.push_back( pts[i] );
    out.push_back( pts.back() );
    return out;
}

} // anonymous namespace

std::vector<Path> Planner::plan( Point start, Point target )
{
    buildNodes( start, target );
    buildEdges();

    std::vector<Path> result;
    std::vector<double> edgeMul( m_edgeList.size(), 1.0 );

    for( int k = 0; k < m_params.kPaths; ++k )
    {
        std::vector<int> nodes = aStar( 0, 1, edgeMul );
        if( nodes.empty() )
            break;

        Path p;
        std::vector<Point> raw;
        for( int idx : nodes )
            raw.push_back( m_nodes[idx] );
        p.waypoints = simplify( raw );

        // Skip if identical to a path we already have (happens when no genuinely
        // distinct alternative exists — we then return fewer than kPaths).
        bool dup = false;
        for( const Path& q : result )
        {
            if( q.waypoints.size() != p.waypoints.size() )
                continue;
            bool same = true;
            for( size_t i = 0; i < p.waypoints.size(); ++i )
                if( dist( p.waypoints[i], q.waypoints[i] ) > 1e-6 ) { same = false; break; }
            if( same ) { dup = true; break; }
        }

        // Cost = sum of *base* edge weights (without the reuse penalty).
        double cost = 0.0;
        for( size_t i = 0; i + 1 < nodes.size(); ++i )
        {
            int u = nodes[i], v = nodes[i + 1];
            for( const Edge& e : m_adj[u] )
                if( e.to == v ) { cost += m_edgeList[e.id].weight; break; }
        }
        p.cost = cost;
        if( !dup )
            result.push_back( std::move( p ) );

        // Penalize this path's edges so the next A* finds a different homotopy.
        for( size_t i = 0; i + 1 < nodes.size(); ++i )
        {
            int u = nodes[i], v = nodes[i + 1];
            for( const Edge& e : m_adj[u] )
                if( e.to == v ) { edgeMul[e.id] *= m_params.reusePenalty; break; }
        }
    }

    std::sort( result.begin(), result.end(),
               []( const Path& a, const Path& b ) { return a.cost < b.cost; } );
    return result;
}

} // namespace gplan
