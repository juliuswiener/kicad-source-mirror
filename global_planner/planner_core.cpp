// planner_core.cpp  — see planner_core.h (v2: multi-layer + vias)

#include "planner_core.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace gplan {

// ---------------------------------------------------------------------------
// Self-contained 2D geometry helpers
// ---------------------------------------------------------------------------
namespace {

inline double dot( Point a, Point b )  { return a.x * b.x + a.y * b.y; }
inline double cross( Point a, Point b ){ return a.x * b.y - a.y * b.x; }
inline Point  sub( Point a, Point b )  { return { a.x - b.x, a.y - b.y }; }
inline double len( Point a )           { return std::sqrt( a.x * a.x + a.y * a.y ); }
inline double dist( Point a, Point b ) { return len( sub( a, b ) ); }

double distPointSeg( Point p, Point a, Point b )
{
    Point ab = sub( b, a );
    double L2 = dot( ab, ab );
    if( L2 < 1e-12 )
        return dist( p, a );
    double t = std::clamp( dot( sub( p, a ), ab ) / L2, 0.0, 1.0 );
    return dist( p, { a.x + ab.x * t, a.y + ab.y * t } );
}

bool segsIntersect( Point a, Point b, Point c, Point d )
{
    auto sgn = []( double v ) { return ( v > 0 ) - ( v < 0 ); };
    double d1 = cross( sub( d, c ), sub( a, c ) );
    double d2 = cross( sub( d, c ), sub( b, c ) );
    double d3 = cross( sub( b, a ), sub( c, a ) );
    double d4 = cross( sub( b, a ), sub( d, a ) );
    return sgn( d1 ) != sgn( d2 ) && sgn( d3 ) != sgn( d4 );
}

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
        if( ( ( poly[i].y > p.y ) != ( poly[j].y > p.y ) ) &&
            ( p.x < ( poly[j].x - poly[i].x ) * ( p.y - poly[i].y )
                        / ( poly[j].y - poly[i].y ) + poly[i].x ) )
            in = !in;
    return in;
}

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

inline double distPointPolygon( Point p, const Polygon& poly )
{
    return distSegPolygon( p, p, poly );
}

// T6 — exact test: does segment a-b pass through the INTERIOR of a convex CCW
// polygon? (Liang-Barsky / Cyrus-Beck half-plane clip — resolution-independent,
// replaces point sampling.) Interior of a CCW poly = left of every edge. We clip
// the parameter t in [0,1] against each edge's half-plane; a positive-length
// surviving interval means the segment crosses the interior. Grazing along the
// boundary yields a zero-length interval and is NOT counted (the block polygons
// are already shrunk by ~margin, so tangent paths sit just outside).
inline bool segHitsConvex( Point a, Point b, const Polygon& poly )
{
    Point d = sub( b, a );
    double t0 = 0.0, t1 = 1.0;
    size_t n = poly.size();
    for( size_t i = 0, j = n - 1; i < n; j = i++ )
    {
        Point e   = sub( poly[i], poly[j] );          // CCW edge j->i, interior on left
        double num = cross( e, sub( a, poly[j] ) );    // f(0): >=0 means inside this half-plane
        double den = cross( e, d );                    // df/dt
        if( std::fabs( den ) < 1e-12 )
        {
            if( num < 0 ) return false;                // parallel and fully outside -> no hit
        }
        else
        {
            double t = -num / den;
            if( den > 0 ) t0 = std::max( t0, t );      // entering interior
            else          t1 = std::min( t1, t );      // leaving interior
            if( t0 > t1 ) return false;
        }
    }
    return ( t1 - t0 ) > 1e-9;                          // positive-length interior crossing
}

double signedArea( const Polygon& poly )
{
    double a = 0;
    size_t n = poly.size();
    for( size_t i = 0, j = n - 1; i < n; j = i++ )
        a += cross( poly[j], poly[i] );
    return a / 2.0;
}

Polygon asCCW( Polygon poly )
{
    if( signedArea( poly ) < 0 )
        std::reverse( poly.begin(), poly.end() );
    return poly;
}

Point lineIntersect( Point p1, Point d1, Point p2, Point d2 )
{
    double den = cross( d1, d2 );
    if( std::fabs( den ) < 1e-12 )
        return p1;
    double t = cross( sub( p2, p1 ), d2 ) / den;
    return { p1.x + d1.x * t, p1.y + d1.y * t };
}

// Convex hull (Andrew's monotone chain). Defensive: callers may pass a slightly
// non-convex fixed hull; we hull it so offsetConvex() stays well-defined.
Polygon convexHull( Polygon pts )
{
    size_t n = pts.size();
    if( n < 3 )
        return pts;
    std::sort( pts.begin(), pts.end(),
               []( Point a, Point b ) { return a.x < b.x || ( a.x == b.x && a.y < b.y ); } );
    std::vector<Point> h( 2 * n );
    int k = 0;
    auto crossOA = []( Point o, Point a, Point b )
    { return cross( sub( a, o ), sub( b, o ) ); };
    for( size_t i = 0; i < n; ++i )                       // lower hull
    {
        while( k >= 2 && crossOA( h[k - 2], h[k - 1], pts[i] ) <= 0 ) k--;
        h[k++] = pts[i];
    }
    for( size_t i = n - 1, t = k + 1; i > 0; --i )        // upper hull
    {
        while( k >= (int) t && crossOA( h[k - 2], h[k - 1], pts[i - 1] ) <= 0 ) k--;
        h[k++] = pts[i - 1];
    }
    h.resize( k - 1 );
    return h;
}

// Offset a convex CCW polygon outward by d (sharp corners).
Polygon offsetConvex( const Polygon& ccw, double d )
{
    size_t n = ccw.size();
    if( n < 3 )
        return ccw;
    std::vector<Point> offA( n ), offDir( n );
    for( size_t i = 0; i < n; ++i )
    {
        Point a = ccw[i], b = ccw[( i + 1 ) % n];
        Point dir = sub( b, a );
        double l = len( dir );
        if( l < 1e-9 ) l = 1;
        dir = { dir.x / l, dir.y / l };
        Point outward = { dir.y, -dir.x };
        offA[i]   = { a.x + outward.x * d, a.y + outward.y * d };
        offDir[i] = dir;
    }
    Polygon out( n );
    for( size_t i = 0; i < n; ++i )
    {
        size_t prev = ( i + n - 1 ) % n;
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
    // T3 — defensive parameter validation. Clamp rather than throw so a caller
    // mistake degrades gracefully instead of aborting a routing run.
    if( m_params.layers.empty() )
        m_params.layers = { 0 };
    if( m_params.kPaths < 1 )       m_params.kPaths = 1;
    if( m_params.viaCost < 0 )      m_params.viaCost = 0;
    if( m_params.clearance < 0 )    m_params.clearance = 0;
    if( m_params.trackWidth <= 0 )  m_params.trackWidth = 1;   // nm; avoid 0-width margin
    if( m_params.reusePenalty < 1 ) m_params.reusePenalty = 1; // <1 would reward reuse

    buildLayers();
}

int Planner::stackPos( int layer ) const
{
    for( size_t i = 0; i < m_params.layers.size(); ++i )
        if( m_params.layers[i] == layer )
            return static_cast<int>( i );
    return -1;
}

void Planner::buildLayers()
{
    const double margin = m_params.clearance + m_params.trackWidth / 2.0;
    const double eps    = margin * 0.02;

    m_layerData.assign( m_params.layers.size(), {} );

    for( const Obstacle& ob : m_obstacles )
    {
        int sp = stackPos( ob.layer );
        if( sp < 0 )
            continue;                       // obstacle on a layer we don't route
        LayerData& ld = m_layerData[sp];
        if( ob.fixed )
        {
            Polygon ccw = asCCW( convexHull( ob.poly ) );  // defensive: ensure convex
            if( ccw.size() < 3 )
                continue;                                  // degenerate, ignore
            ld.fixedOrig.push_back( ccw );
            ld.fixedInflated.push_back( offsetConvex( ccw, margin ) );
            ld.fixedBlock.push_back( offsetConvex( ccw, margin - eps ) );
        }
        else
        {
            ld.movable.push_back( ob.poly );
        }
    }
}

void Planner::bumpCongestion( Point where, double radius, double factor )
{
    std::lock_guard<std::mutex> lk( m_mutex );   // T1
    m_bumps.push_back( { where, radius, factor } );
}

void Planner::clearCongestion()
{
    std::lock_guard<std::mutex> lk( m_mutex );   // T1
    m_bumps.clear();
}

void Planner::buildNodes( Point start, int sL, Point target, int tL )
{
    m_nodes.clear();

    // Start and target are added first and are NEVER filtered: they legitimately
    // sit on/inside their own pad hulls.
    m_srcIdx = static_cast<int>( m_nodes.size() );
    m_nodes.push_back( { start, sL } );
    m_dstIdx = static_cast<int>( m_nodes.size() );
    m_nodes.push_back( { target, tL } );

    // Candidate (x,y) columns: start, target, and every fixed inflated corner
    // from every layer. Each column is replicated on every routed layer (so a via
    // can land there) — but only where the position is actually clear of fixed
    // copper on that layer. A corner that falls inside an overlapping obstacle is
    // not a valid placement and must be dropped (else it would open a hole).
    std::vector<Point> xy = { start, target };
    for( const LayerData& ld : m_layerData )
        for( const Polygon& hull : ld.fixedInflated )
            for( const Point& v : hull )
                xy.push_back( v );

    auto insideAnyBlock = [&]( Point p, int sp )
    {
        for( const Polygon& b : m_layerData[sp].fixedBlock )
            if( pointInPolygon( p, b ) )
                return true;
        return false;
    };

    for( int layer : m_params.layers )
    {
        int sp = stackPos( layer );
        for( const Point& p : xy )
        {
            // Don't duplicate the start/target nodes already added.
            if( ( layer == sL && dist( p, start ) < 1e-9 )
                || ( layer == tL && dist( p, target ) < 1e-9 ) )
                continue;
            if( sp >= 0 && insideAnyBlock( p, sp ) )
                continue;
            m_nodes.push_back( { p, layer } );
        }
    }
}

bool Planner::edgeBlocked( Point a, Point b, int layer ) const
{
    int sp = stackPos( layer );
    if( sp < 0 )
        return true;
    for( const Polygon& block : m_layerData[sp].fixedBlock )
    {
        // An endpoint inside this hull means we are legitimately leaving/entering
        // the obstacle it represents (e.g. the source/target pad) — it must not
        // block edges incident to that endpoint.
        if( pointInPolygon( a, block ) || pointInPolygon( b, block ) )
            continue;
        if( segHitsConvex( a, b, block ) )             // T6: exact, no sampling
            return true;
    }
    return false;
}

bool Planner::viaSiteClear( Point p, int layer ) const
{
    int sp = stackPos( layer );
    if( sp < 0 )
        return false;
    double viaMargin = m_params.viaClearance + m_params.viaDiameter / 2.0;
    for( const Polygon& hull : m_layerData[sp].fixedOrig )
        if( distPointPolygon( p, hull ) < viaMargin )
            return false;
    return true;
}

double Planner::edgeWeight( Point a, Point b, int layer ) const
{
    const double margin = m_params.clearance + m_params.trackWidth / 2.0;
    const double pitch  = m_params.trackWidth + m_params.clearance;
    const double base   = dist( a, b );
    const LayerData& ld = m_layerData[stackPos( layer )];

    double dFix = std::numeric_limits<double>::max();
    for( const Polygon& hull : ld.fixedOrig )
        dFix = std::min( dFix, distSegPolygon( a, b, hull ) );
    if( dFix == std::numeric_limits<double>::max() )
        dFix = 10.0 * pitch;

    double gap      = 2.0 * dFix;
    double capacity = std::max( 1.0, std::floor( gap / pitch ) );

    double usage = 0.0;
    for( const Polygon& mv : ld.movable )
        if( distSegPolygon( a, b, mv ) < margin )
            usage += 1.0;
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

    auto addEdge = [&]( int i, int j, double w )
    {
        int id = static_cast<int>( m_edgeList.size() );
        m_edgeList.push_back( { i, j, w } );
        m_adj[i].push_back( { j, id } );
        m_adj[j].push_back( { i, id } );
    };

    for( int i = 0; i < n; ++i )
    {
        for( int j = i + 1; j < n; ++j )
        {
            const Node& A = m_nodes[i];
            const Node& B = m_nodes[j];

            if( A.layer == B.layer )
            {
                // Intra-layer visibility edge.
                if( dist( A.p, B.p ) < 1e-9 )
                    continue;
                if( edgeBlocked( A.p, B.p, A.layer ) )
                    continue;
                addEdge( i, j, edgeWeight( A.p, B.p, A.layer ) );
            }
            else if( dist( A.p, B.p ) < 1e-9 )
            {
                // Same (x,y), different layer -> candidate via, only between
                // layers adjacent in the stack, clear on both.
                if( std::abs( stackPos( A.layer ) - stackPos( B.layer ) ) != 1 )
                    continue;
                if( viaSiteClear( A.p, A.layer ) && viaSiteClear( B.p, B.layer ) )
                    addEdge( i, j, m_params.viaCost );
            }
        }
    }
}

std::vector<int> Planner::aStar( int src, int dst, const std::vector<double>& mul ) const
{
    int n = static_cast<int>( m_nodes.size() );
    std::vector<double> g( n, std::numeric_limits<double>::max() );
    std::vector<int>    prev( n, -1 );

    auto h = [&]( int i ) { return dist( m_nodes[i].p, m_nodes[dst].p ); };

    using QE = std::pair<double, int>;
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
            continue;
        for( const Edge& e : m_adj[u] )
        {
            double ng = g[u] + m_edgeList[e.id].weight * mul[e.id];
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
        return path;
    for( int u = dst; u != -1; u = prev[u] )
        path.push_back( u );
    std::reverse( path.begin(), path.end() );
    return path;
}

namespace {

// Drop interior points collinear with neighbours on the SAME layer (keep vias).
std::vector<Waypoint> simplify( const std::vector<Waypoint>& wps )
{
    if( wps.size() <= 2 )
        return wps;
    std::vector<Waypoint> out;
    out.push_back( wps.front() );
    for( size_t i = 1; i + 1 < wps.size(); ++i )
    {
        bool sameLayer = ( wps[i].layer == out.back().layer )
                         && ( wps[i].layer == wps[i + 1].layer );
        if( !sameLayer
            || distPointSeg( wps[i].p, out.back().p, wps[i + 1].p ) > 1e-6 )
            out.push_back( wps[i] );
    }
    out.push_back( wps.back() );
    return out;
}

} // anonymous namespace

std::vector<Path> Planner::plan( Point start, int sL, Point target, int tL )
{
    std::lock_guard<std::mutex> lk( m_mutex );   // T1: guards graph rebuild + m_bumps
    buildNodes( start, sL, target, tL );
    buildEdges();

    std::vector<Path> result;
    if( m_srcIdx < 0 || m_dstIdx < 0 )
        return result;

    // Trivial: start and target coincide (same point, same layer).
    if( sL == tL && dist( start, target ) < 1e-9 )
    {
        Path p;
        p.waypoints = { { start, sL } };
        result.push_back( p );
        return result;
    }

    std::vector<double> mul( m_edgeList.size(), 1.0 );

    for( int k = 0; k < m_params.kPaths; ++k )
    {
        std::vector<int> nodes = aStar( m_srcIdx, m_dstIdx, mul );
        if( nodes.empty() )
            break;

        Path p;
        std::vector<Waypoint> raw;
        for( int idx : nodes )
            raw.push_back( { m_nodes[idx].p, m_nodes[idx].layer } );
        p.waypoints = simplify( raw );

        bool dup = false;
        for( const Path& q : result )
        {
            if( q.waypoints.size() != p.waypoints.size() )
                continue;
            bool same = true;
            for( size_t i = 0; i < p.waypoints.size(); ++i )
                if( dist( p.waypoints[i].p, q.waypoints[i].p ) > 1e-6
                    || p.waypoints[i].layer != q.waypoints[i].layer )
                { same = false; break; }
            if( same ) { dup = true; break; }
        }

        double cost = 0.0;
        for( size_t i = 0; i + 1 < nodes.size(); ++i )
            for( const Edge& e : m_adj[nodes[i]] )
                if( e.to == nodes[i + 1] ) { cost += m_edgeList[e.id].weight; break; }
        p.cost = cost;

        if( !dup )
            result.push_back( std::move( p ) );

        for( size_t i = 0; i + 1 < nodes.size(); ++i )
            for( const Edge& e : m_adj[nodes[i]] )
                if( e.to == nodes[i + 1] ) { mul[e.id] *= m_params.reusePenalty; break; }
    }

    std::sort( result.begin(), result.end(),
               []( const Path& a, const Path& b ) { return a.cost < b.cost; } );
    return result;
}

std::vector<Path> Planner::plan( Point start, Point target )
{
    int l = m_params.layers.front();
    return plan( start, l, target, l );
}

} // namespace gplan
