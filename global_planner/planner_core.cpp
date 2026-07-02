// planner_core.cpp  — see planner_core.h (v2: multi-layer + vias)

#include "planner_core.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <set>
#include <tuple>

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

// T5 — AABB helpers for exact spatial culling (only skip a hull when its box is
// provably too far → results stay bit-identical to the brute-force version).
inline double segLo( double a, double b ) { return a < b ? a : b; }
inline double segHi( double a, double b ) { return a > b ? a : b; }
// Distance between two axis-aligned boxes (0 if they overlap).
inline double aabbDist( double ax0, double ay0, double ax1, double ay1,
                        double bx0, double by0, double bx1, double by1 )
{
    double dx = std::max( 0.0, std::max( bx0 - ax1, ax0 - bx1 ) );
    double dy = std::max( 0.0, std::max( by0 - ay1, ay0 - by1 ) );
    return std::sqrt( dx * dx + dy * dy );
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

// T-GRID — uniform spatial hash. Cell size ~ average hull AABB extent, so a
// typical hull touches O(1) cells and a typical query returns O(1) candidates
// regardless of total hull count m.
int Planner::SpatialGrid::cellX( double x ) const
{ return std::clamp( (int) ( ( x - ox ) / cell ), 0, nx - 1 ); }
int Planner::SpatialGrid::cellY( double y ) const
{ return std::clamp( (int) ( ( y - oy ) / cell ), 0, ny - 1 ); }

void Planner::SpatialGrid::build( const std::vector<AABB>& boxes )
{
    epoch.assign( boxes.size(), 0 );
    epochCounter = 0;
    cells.clear();

    if( boxes.empty() )
    {
        nx = ny = 1;
        cells.assign( 1, {} );
        return;
    }

    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300, avgW = 0, avgH = 0;
    for( const AABB& b : boxes )
    {
        x0 = std::min( x0, b.x0 ); y0 = std::min( y0, b.y0 );
        x1 = std::max( x1, b.x1 ); y1 = std::max( y1, b.y1 );
        avgW += b.x1 - b.x0; avgH += b.y1 - b.y0;
    }
    avgW /= boxes.size(); avgH /= boxes.size();
    double sizeHeuristic = std::max( avgW, avgH );
    if( sizeHeuristic < 1e-9 )
        // Degenerate/point boxes (T-NEIGHBOR's node grid): avgW/avgH are 0, so
        // fall back to a point-density heuristic — typical nearest-neighbour
        // spacing for boxes.size() points spread over the bounding area.
        sizeHeuristic = ( ( x1 - x0 ) + ( y1 - y0 ) ) / ( 2.0 * std::sqrt( (double) boxes.size() ) );
    cell = std::max( 1.0, sizeHeuristic );   // cell ~ typical hull size / point spacing
    ox = x0; oy = y0;

    auto dims = [&]()
    {
        nx = std::max( 1, (int) std::ceil( ( x1 - x0 ) / cell ) + 1 );
        ny = std::max( 1, (int) std::ceil( ( y1 - y0 ) / cell ) + 1 );
    };
    dims();
    // Guard against a pathological aspect ratio (e.g. one huge board-edge box
    // dragging avgW/avgH tiny) blowing up cell count / memory.
    long long total = (long long) nx * (long long) ny;
    if( total > 4'000'000 )
    {
        cell *= std::sqrt( (double) total / 4'000'000.0 );
        dims();
    }
    cells.assign( (size_t) nx * (size_t) ny, {} );

    for( size_t i = 0; i < boxes.size(); ++i )
    {
        const AABB& b = boxes[i];
        int cx0 = cellX( b.x0 ), cx1 = cellX( b.x1 );
        int cy0 = cellY( b.y0 ), cy1 = cellY( b.y1 );
        for( int cy = cy0; cy <= cy1; ++cy )
            for( int cx = cx0; cx <= cx1; ++cx )
                cells[ (size_t) cy * nx + cx ].push_back( (int) i );
    }
}

void Planner::SpatialGrid::queryInto( double x0, double y0, double x1, double y1,
                                      std::vector<int>& out ) const
{
    ++epochCounter;
    int cx0 = cellX( x0 ), cx1 = cellX( x1 );
    int cy0 = cellY( y0 ), cy1 = cellY( y1 );
    if( cx0 > cx1 ) std::swap( cx0, cx1 );
    if( cy0 > cy1 ) std::swap( cy0, cy1 );
    for( int cy = cy0; cy <= cy1; ++cy )
        for( int cx = cx0; cx <= cx1; ++cx )
            for( int idx : cells[ (size_t) cy * nx + cx ] )
                if( epoch[idx] != epochCounter )
                { epoch[idx] = epochCounter; out.push_back( idx ); }
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

    auto polyAABB = []( const Polygon& p ) -> AABB {
        double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
        for( const Point& v : p )
        { x0 = std::min(x0,v.x); y0 = std::min(y0,v.y); x1 = std::max(x1,v.x); y1 = std::max(y1,v.y); }
        return { x0, y0, x1, y1 };
    };

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
            Polygon block = offsetConvex( ccw, margin - eps );
            ld.fixedOrigBox.push_back( polyAABB( ccw ) );
            ld.fixedBlockBox.push_back( polyAABB( block ) );
            ld.fixedOrig.push_back( std::move( ccw ) );
            ld.fixedInflated.push_back( offsetConvex( ld.fixedOrig.back(), margin ) );
            ld.fixedBlock.push_back( std::move( block ) );
        }
        else
        {
            ld.movableBox.push_back( polyAABB( ob.poly ) );
            ld.movable.push_back( ob.poly );
        }
    }

    // T-GRID: index each layer's hulls once (see SpatialGrid comment in the
    // header for why fixedGrid is built over fixedBlockBox, the outer box).
    for( LayerData& ld : m_layerData )
    {
        ld.fixedGrid.build( ld.fixedBlockBox );
        ld.movableGrid.build( ld.movableBox );
    }
}

void Planner::bumpCongestion( Point where, double radius, double factor )
{
    std::lock_guard<std::mutex> lk( m_mutex );   // T1
    m_bumps.push_back( { where, radius, factor } );
    m_graphDirty = true;                         // T-CACHE: bumps feed edgeWeight
}

void Planner::clearCongestion()
{
    std::lock_guard<std::mutex> lk( m_mutex );   // T1
    if( !m_bumps.empty() )
        m_graphDirty = true;
    m_bumps.clear();
}

void Planner::setRegion( const BBox& region )
{
    std::lock_guard<std::mutex> lk( m_mutex );
    if( m_hasRegion && region.x0 == m_region.x0 && region.y0 == m_region.y0
        && region.x1 == m_region.x1 && region.y1 == m_region.y1 )
        return;                                  // unchanged -> keep cache warm
    m_hasRegion = true;
    m_region = region;
    m_graphDirty = true;
}

void Planner::clearRegion()
{
    std::lock_guard<std::mutex> lk( m_mutex );
    if( !m_hasRegion )
        return;
    m_hasRegion = false;
    m_graphDirty = true;
}

int Planner::graphBuildCount() const
{
    std::lock_guard<std::mutex> lk( m_mutex );
    return m_graphBuilds;
}

// point-in-polygon implies point-in-AABB, so an unpadded point query is
// exact — no need to grow the search box.
bool Planner::insideAnyBlock( Point p, int sp ) const
{
    const LayerData& ld = m_layerData[sp];
    m_queryBuf.clear();
    ld.fixedGrid.queryInto( p.x, p.y, p.x, p.y, m_queryBuf );
    for( int i : m_queryBuf )
        if( pointInPolygon( p, ld.fixedBlock[i] ) )
            return true;
    return false;
}

void Planner::buildCornerNodes()
{
    m_nodes.clear();

    // Candidate (x,y) columns: every fixed inflated corner from every layer
    // (region-filtered, T-REGION). Each column is replicated on every routed
    // layer (so a via can land there) — but only where the position is actually
    // clear of fixed copper on that layer. A corner that falls inside an
    // overlapping obstacle is not a valid placement and must be dropped (else
    // it would open a hole). Start/target are NOT here: they are appended as a
    // small per-plan overlay so this corner graph is cacheable (T-CACHE).
    std::vector<Point> xy;
    for( const LayerData& ld : m_layerData )
        for( const Polygon& hull : ld.fixedInflated )
            for( const Point& v : hull )
            {
                if( m_hasRegion
                    && ( v.x < m_region.x0 || v.x > m_region.x1
                         || v.y < m_region.y0 || v.y > m_region.y1 ) )
                    continue;
                xy.push_back( v );
            }

    for( int layer : m_params.layers )
    {
        int sp = stackPos( layer );
        for( const Point& p : xy )
        {
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
    const LayerData& ld = m_layerData[sp];
    double sx0 = segLo(a.x,b.x), sy0 = segLo(a.y,b.y), sx1 = segHi(a.x,b.x), sy1 = segHi(a.y,b.y);

    m_queryBuf.clear();
    ld.fixedGrid.queryInto( sx0, sy0, sx1, sy1, m_queryBuf );        // T-GRID: candidates only
    for( int i : m_queryBuf )
    {
        const AABB& bx = ld.fixedBlockBox[i];                       // T5: exact cull (grid is coarse)
        if( aabbDist( sx0, sy0, sx1, sy1, bx.x0, bx.y0, bx.x1, bx.y1 ) > 0.0 )
            continue;
        const Polygon& block = ld.fixedBlock[i];
        // An endpoint inside this hull means we are legitimately leaving/entering
        // the obstacle it represents (e.g. the source/target pad).
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
    const LayerData& ld = m_layerData[sp];

    m_queryBuf.clear();
    // Search box padded by viaMargin: anything farther than that can't matter.
    ld.fixedGrid.queryInto( p.x - viaMargin, p.y - viaMargin,
                            p.x + viaMargin, p.y + viaMargin, m_queryBuf );
    for( int i : m_queryBuf )
    {
        const AABB& bx = ld.fixedOrigBox[i];                        // T5: exact cull
        if( aabbDist( p.x, p.y, p.x, p.y, bx.x0, bx.y0, bx.x1, bx.y1 ) >= viaMargin )
            continue;
        if( distPointPolygon( p, ld.fixedOrig[i] ) < viaMargin )
            return false;
    }
    return true;
}

double Planner::edgeWeight( Point a, Point b, int layer ) const
{
    const double margin = m_params.clearance + m_params.trackWidth / 2.0;
    const double pitch  = m_params.trackWidth + m_params.clearance;
    const double base   = dist( a, b );
    const LayerData& ld = m_layerData[stackPos( layer )];
    double sx0 = segLo(a.x,b.x), sy0 = segLo(a.y,b.y), sx1 = segHi(a.x,b.x), sy1 = segHi(a.y,b.y);

    // dFix wants the nearest fixed hull with NO fixed search radius (unlike
    // edgeBlocked/viaSiteClear/usage below), so the grid query starts at one
    // cell and doubles until it finds candidates — exact same result as the
    // old unbounded linear scan, just without touching every one of the m
    // hulls when the nearby ones already answer it.
    // Corridor width (§2.4): nearest fixed obstacle on EACH side of the a->b
    // line, so `gap` is the width of the channel between the two bounding
    // obstacles instead of 2x the single nearest one. Side is judged by the
    // obstacle's bbox center — a heuristic for congestion ranking only.
    double dSide[2] = { std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max() };
    {
        double r = std::max( pitch, ld.fixedGrid.cell );
        for( int ring = 0; ring < 12; ++ring )
        {
            m_queryBuf.clear();
            ld.fixedGrid.queryInto( sx0 - r, sy0 - r, sx1 + r, sy1 + r, m_queryBuf );
            if( !m_queryBuf.empty() )
                break;
            if( ld.fixedGrid.nx == 1 && ld.fixedGrid.ny == 1 )
                break;                                  // whole grid is one cell; no point growing
            r *= 2;
        }
        const double ex = b.x - a.x, ey = b.y - a.y;
        for( int i : m_queryBuf )
        {
            const AABB& bx = ld.fixedOrigBox[i];
            double cross = ex * ( ( bx.y0 + bx.y1 ) * 0.5 - a.y )
                         - ey * ( ( bx.x0 + bx.x1 ) * 0.5 - a.x );
            int side = ( cross >= 0.0 ) ? 0 : 1;
            if( aabbDist( sx0, sy0, sx1, sy1, bx.x0, bx.y0, bx.x1, bx.y1 ) >= dSide[side] )
                continue;                                           // T5: prune per side
            dSide[side] = std::min( dSide[side], distSegPolygon( a, b, ld.fixedOrig[i] ) );
        }
    }
    // Empty side -> open field there; same 10*pitch cap the old code used
    // when nothing was found at all (capacity beyond ~10 tracks is moot).
    for( double& d : dSide )
        if( d == std::numeric_limits<double>::max() )
            d = 10.0 * pitch;

    double dFix     = std::min( dSide[0], dSide[1] );
    double gap      = dSide[0] + dSide[1];
    double capacity = std::max( 1.0, std::floor( gap / pitch ) );

    double usage = 0.0;
    m_queryBuf.clear();
    ld.movableGrid.queryInto( sx0 - margin, sy0 - margin, sx1 + margin, sy1 + margin, m_queryBuf );
    for( int i : m_queryBuf )
    {
        const AABB& bx = ld.movableBox[i];                          // T5: exact cull
        if( aabbDist( sx0, sy0, sx1, sy1, bx.x0, bx.y0, bx.x1, bx.y1 ) >= margin )
            continue;
        if( distSegPolygon( a, b, ld.movable[i] ) < margin )
            usage += 1.0;
    }
    for( const CongestionBump& bp : m_bumps )
        if( distPointSeg( bp.center, a, b ) < bp.radius )
            usage += bp.factor;

    double load       = usage / capacity;
    double congestion = m_params.wCongestion * load * load * base;
    double tightness  = m_params.wTightness * ( margin / std::max( dFix, margin ) ) * base;

    return base + congestion + tightness;
}

void Planner::connectPair( int i, int j )
{
    const Node& A = m_nodes[i];
    const Node& B = m_nodes[j];

    auto addEdge = [&]( double w )
    {
        int id = static_cast<int>( m_edgeList.size() );
        m_edgeList.push_back( { i, j, w } );
        m_adj[i].push_back( { j, id } );
        m_adj[j].push_back( { i, id } );
    };

    if( A.layer == B.layer )
    {
        // Intra-layer visibility edge.
        if( dist( A.p, B.p ) < 1e-9 )
            return;
        if( edgeBlocked( A.p, B.p, A.layer ) )
            return;
        addEdge( edgeWeight( A.p, B.p, A.layer ) );
    }
    else if( dist( A.p, B.p ) < 1e-9 )
    {
        // Same (x,y), different layer -> candidate via, only between
        // layers adjacent in the stack, clear on both.
        if( std::abs( stackPos( A.layer ) - stackPos( B.layer ) ) != 1 )
            return;
        if( viaSiteClear( A.p, A.layer ) && viaSiteClear( B.p, B.layer ) )
            addEdge( m_params.viaCost );
    }
}

void Planner::buildCornerEdges()
{
    int n = static_cast<int>( m_nodes.size() );
    m_adj.assign( n, {} );
    m_edgeList.clear();

    // T-NEIGHBOR: buildEdges used to test EVERY node pair (O(n^2) candidates) —
    // T-GRID (above) only sped up the per-candidate obstacle scan, not this
    // outer enumeration. Bound candidate generation to spatially-nearby pairs
    // instead: each node queries an expanding ring (via a grid over node XY
    // positions) until it has enough neighbours, or the ring covers the whole
    // board. Start/target are not in this graph at all (T-CACHE overlay in
    // plan() connects them exhaustively — a free direct sightline is never
    // missed regardless of distance). A pair found from EITHER side's query is
    // tested at most once (seenPairs dedup).
    std::vector<AABB> nodeBoxes( n );
    for( int i = 0; i < n; ++i )
        nodeBoxes[i] = { m_nodes[i].p.x, m_nodes[i].p.y, m_nodes[i].p.x, m_nodes[i].p.y };
    SpatialGrid nodeGrid;
    nodeGrid.build( nodeBoxes );

    double boardDiag = 0.0;
    {
        double bx0 = 1e300, by0 = 1e300, bx1 = -1e300, by1 = -1e300;
        for( int i = 0; i < n; ++i )
        {
            bx0 = std::min( bx0, m_nodes[i].p.x ); by0 = std::min( by0, m_nodes[i].p.y );
            bx1 = std::max( bx1, m_nodes[i].p.x ); by1 = std::max( by1, m_nodes[i].p.y );
        }
        boardDiag = dist( { bx0, by0 }, { bx1, by1 } );
    }
    const int MIN_NEIGHBORS = 24;   // generous floor; keeps homotopy enumeration intact

    std::set<int64_t> seenPairs;
    auto pairKey = []( int a, int b )
    { if( a > b ) std::swap( a, b ); return (int64_t) a * 10'000'000LL + b; };

    std::vector<int> nbuf;
    for( int i = 0; i < n; ++i )
    {
        double r = std::max( nodeGrid.cell, 1.0 ) * 2.0;
        for( int ring = 0; ring < 24; ++ring )
        {
            nbuf.clear();
            nodeGrid.queryInto( m_nodes[i].p.x - r, m_nodes[i].p.y - r,
                                m_nodes[i].p.x + r, m_nodes[i].p.y + r, nbuf );
            if( (int) nbuf.size() > MIN_NEIGHBORS || r >= boardDiag )
                break;
            r *= 2.0;
        }
        for( int j : nbuf )
        {
            if( j == i )
                continue;
            int64_t key = pairKey( i, j );
            if( !seenPairs.insert( key ).second )
                continue;                    // already tested from the other side
            connectPair( std::min( i, j ), std::max( i, j ) );
        }
    }
}

std::vector<int> Planner::aStar( int src, int dst, const std::vector<double>& mul ) const
{
    int n = static_cast<int>( m_nodes.size() );
    std::vector<double> g( n, std::numeric_limits<double>::max() );
    std::vector<int>    prev( n, -1 );

    auto h = [&]( int i ) { return dist( m_nodes[i].p, m_nodes[dst].p ); };

    // T-TIEBREAK (2.6): equal-f entries break ties by HIGHER g (stored negated,
    // so std::greater picks it). On a visibility graph many nodes share the same
    // f along an optimal corridor; preferring the deeper node (larger g, hence
    // smaller remaining h) walks straight to the target instead of expanding the
    // whole equal-cost front. Node index is a last key for determinism. Carrying
    // g in the entry also makes the stale-entry check exact (no f - h roundoff).
    using QE = std::tuple<double, double, int>;   // ( f, -g, node )
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> pq;
    g[src] = 0.0;
    pq.push( { h( src ), 0.0, src } );

    while( !pq.empty() )
    {
        auto [f, negG, u] = pq.top();
        pq.pop();
        if( u == dst )
            break;
        if( -negG > g[u] + 1e-9 )
            continue;
        for( const Edge& e : m_adj[u] )
        {
            double ng = g[u] + m_edgeList[e.id].weight * mul[e.id];
            if( ng + 1e-9 < g[e.to] )
            {
                g[e.to] = ng;
                prev[e.to] = u;
                pq.push( { ng + h( e.to ), -ng, e.to } );
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

    // T-CACHE (2.5): the corner-only graph is independent of start/target.
    // Rebuild it only when bumps/region changed since the last plan(); otherwise
    // reuse the cached copy and just overlay this call's start/target nodes.
    if( m_graphDirty )
    {
        buildCornerNodes();
        buildCornerEdges();
        m_cornerNodes    = m_nodes;
        m_cornerAdj      = m_adj;
        m_cornerEdgeList = m_edgeList;
        m_graphDirty = false;
        ++m_graphBuilds;
    }
    else
    {
        m_nodes    = m_cornerNodes;
        m_adj      = m_cornerAdj;
        m_edgeList = m_cornerEdgeList;
    }

    // Per-plan overlay: start + target (NEVER filtered — they legitimately sit
    // on/inside their own pad hulls), plus their (x,y) columns replicated on
    // the other routed layers (so a via can land there) where clear of fixed
    // copper.
    const int cornerCount = static_cast<int>( m_nodes.size() );
    m_srcIdx = cornerCount;
    m_nodes.push_back( { start, sL } );
    m_dstIdx = cornerCount + 1;
    m_nodes.push_back( { target, tL } );

    for( int layer : m_params.layers )
    {
        int sp = stackPos( layer );
        for( const Point& p : { start, target } )
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

    // Connect every overlay node against ALL nodes (exhaustive — at most
    // 2*layers overlay nodes, O(layers*n) pair tests; a free direct sightline
    // from start/target is never missed regardless of distance).
    m_adj.resize( m_nodes.size() );
    for( int i = cornerCount; i < (int) m_nodes.size(); ++i )
        for( int j = 0; j < i; ++j )
            connectPair( j, i );

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

    // ---- T4: Yen's k-shortest loopless paths (guarantees distinct routes) ----
    const double      INF = 1e18;
    const size_t      nEdges = m_edgeList.size();
    std::vector<double> ones( nEdges, 1.0 );

    auto edgeId = [&]( int u, int v ) -> int {
        for( const Edge& e : m_adj[u] ) if( e.to == v ) return e.id;
        return -1;
    };
    auto pathCost = [&]( const std::vector<int>& p ) {
        double c = 0.0;
        for( size_t i = 0; i + 1 < p.size(); ++i )
        { int id = edgeId( p[i], p[i + 1] ); if( id >= 0 ) c += m_edgeList[id].weight; }
        return c;
    };

    std::vector<std::vector<int>>   A;          // accepted shortest paths (node ids)
    std::set<std::vector<int>>      inA;        // dedup
    std::vector<std::pair<double, std::vector<int>>> B;   // candidate spur paths
    std::set<std::vector<int>>      inB;

    std::vector<int> first = aStar( m_srcIdx, m_dstIdx, ones );
    if( !first.empty() )
    {
        A.push_back( first );
        inA.insert( first );

        while( (int) A.size() < m_params.kPaths )
        {
            const std::vector<int>& prev = A.back();
            for( size_t i = 0; i + 1 < prev.size(); ++i )
            {
                int spur = prev[i];
                std::vector<int> root( prev.begin(), prev.begin() + i + 1 );
                std::vector<double> mul = ones;

                // Remove the edge that each already-found path took from this spur,
                // if that path shares the same root — forces a new branch here.
                for( const std::vector<int>& p : A )
                    if( p.size() > i + 1
                        && std::equal( root.begin(), root.end(), p.begin() ) )
                    { int id = edgeId( p[i], p[i + 1] ); if( id >= 0 ) mul[id] = INF; }

                // Remove the root nodes (except the spur) from the graph.
                for( size_t r = 0; r < i; ++r )
                    for( const Edge& e : m_adj[root[r]] )
                        mul[e.id] = INF;

                std::vector<int> spurPath = aStar( spur, m_dstIdx, mul );
                if( spurPath.empty() )
                    continue;

                std::vector<int> total( root.begin(), root.end() - 1 ); // drop dup spur
                total.insert( total.end(), spurPath.begin(), spurPath.end() );

                if( inA.count( total ) || inB.count( total ) )
                    continue;
                B.emplace_back( pathCost( total ), total );
                inB.insert( total );
            }

            if( B.empty() )
                break;
            auto best = std::min_element( B.begin(), B.end(),
                []( const auto& x, const auto& y ) { return x.first < y.first; } );
            A.push_back( best->second );
            inA.insert( best->second );
            inB.erase( best->second );
            B.erase( best );
        }
    }

    // Convert node paths -> waypoint Paths (simplify + cost), dedup by geometry.
    for( const std::vector<int>& nodes : A )
    {
        Path p;
        std::vector<Waypoint> raw;
        for( int idx : nodes )
            raw.push_back( { m_nodes[idx].p, m_nodes[idx].layer } );
        p.waypoints = simplify( raw );
        p.cost = pathCost( nodes );

        bool dup = false;
        for( const Path& q : result )
        {
            if( q.waypoints.size() != p.waypoints.size() ) continue;
            bool same = true;
            for( size_t i = 0; i < p.waypoints.size(); ++i )
                if( dist( p.waypoints[i].p, q.waypoints[i].p ) > 1e-6
                    || p.waypoints[i].layer != q.waypoints[i].layer )
                { same = false; break; }
            if( same ) { dup = true; break; }
        }
        if( !dup )
            result.push_back( std::move( p ) );
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

std::vector<Path> Planner::plan( Point start, int sL, Point target, int tL,
                                 const BBox& region )
{
    setRegion( region );   // no-op (cache stays warm) if unchanged
    return plan( start, sL, target, tL );
}

std::vector<Path> Planner::plan( Point start, Point target, const BBox& region )
{
    int l = m_params.layers.front();
    return plan( start, l, target, l, region );
}

std::vector<Path> Planner::planMultiTerminal( const std::vector<Waypoint>& terminals )
{
    std::vector<Path> tree;
    if( terminals.size() < 2 )
        return tree;

    std::vector<Waypoint> connected = { terminals.front() };
    std::vector<Waypoint> remaining( terminals.begin() + 1, terminals.end() );

    while( !remaining.empty() )
    {
        double bestCost = -1.0;
        size_t bestRemaining = 0;
        Path   bestPath;

        for( size_t r = 0; r < remaining.size(); ++r )
        {
            for( const Waypoint& c : connected )
            {
                std::vector<Path> cand = plan( c.p, c.layer, remaining[r].p, remaining[r].layer );
                if( cand.empty() )
                    continue;
                if( bestCost < 0.0 || cand.front().cost < bestCost )
                {
                    bestCost      = cand.front().cost;
                    bestRemaining = r;
                    bestPath      = std::move( cand.front() );
                }
            }
        }

        if( bestCost < 0.0 )   // no remaining terminal reachable from the tree so far
            break;

        tree.push_back( std::move( bestPath ) );
        connected.push_back( remaining[bestRemaining] );
        remaining.erase( remaining.begin() + bestRemaining );
    }

    return tree;
}

} // namespace gplan
