// pns_bridge.cpp  — see pns_bridge.h
//
// KiCad-linked. Builds inside the KiCad source tree only. The #includes below
// follow the paths used by pcbnew/router/*. If your tree differs, adjust them.
//
// References (verified in this repo):
//   - headless router setup:  qa/tools/pns/pns_log_player.cpp::createRouter()
//   - board+rules loading:    pcbnew/board_loader.cpp::initializeLoadedBoard()
//   - collision read-out:     pcbnew/router/pns_router.cpp::markViolations()

#include "pns_bridge.h"

#include <board.h>
#include <pcb_track.h>
#include <pad.h>
#include <zone.h>
#include <footprint.h>
#include <layer_ids.h>
#include <math/vector2d.h>
#include <geometry/shape_line_chain.h>
#include <settings/settings_manager.h>

#include <router/pns_router.h>
#include <router/pns_kicad_iface.h>
#include <router/pns_routing_settings.h>
#include <router/pns_sizes_settings.h>
#include <router/pns_itemset.h>
#include <router/pns_line.h>
#include <router/pns_segment.h>
#include <router/pns_via.h>
#include <router/pns_node.h>
#include <router/pns_placement_algo.h>

using namespace gbridge;

// A minimal headless iface: PNS_KICAD_IFACE_BASE already stubs the view/commit
// methods, so we only need the net helpers (its base returns -1 / "").
class HeadlessIface : public PNS_KICAD_IFACE_BASE
{
public:
    int GetNetCode( PNS::NET_HANDLE aNet ) const override
    {
        return aNet ? static_cast<NETINFO_ITEM*>( aNet )->GetNetCode() : -1;
    }
    wxString GetNetName( PNS::NET_HANDLE aNet ) const override
    {
        return aNet ? static_cast<NETINFO_ITEM*>( aNet )->GetNetname() : wxString();
    }
};

PnsBridge::PnsBridge() = default;
PnsBridge::~PnsBridge() = default;

// NOTE: PnsBridge::load() lives in pns_bridge_load.cpp — it pulls in
// BOARD_LOADER (the pcbnew kiface). Hosts that load boards themselves link only
// this TU and call attach(), avoiding the kiface dependency.

bool PnsBridge::attach( BOARD* board )
{
    if( !board )
        return false;
    m_board = board;

    m_iface = std::make_unique<HeadlessIface>();
    m_iface->SetBoard( m_board );

    m_router = std::make_unique<PNS::ROUTER>();
    m_router->SetInterface( m_iface.get() );

    // ROUTER::Settings() dereferences m_settings; it must be loaded before
    // SyncWorld()/SetMode() (mirrors qa pns_log_player::createRouter()).
    m_routingSettings = std::make_unique<PNS::ROUTING_SETTINGS>( nullptr, "" );
    m_router->LoadSettings( m_routingSettings.get() );

    m_router->ClearWorld();
    m_router->SyncWorld();

    m_router->Settings().SetMode( PNS::RM_Shove );
    return true;
}

int PnsBridge::pnsLayer( int boardLayer ) const
{
    return m_iface->GetPNSLayerFromBoardLayer( static_cast<PCB_LAYER_ID>( boardLayer ) );
}

// ---------------------------------------------------------------------------
// Obstacle extraction (BOARD -> gplan::Obstacle)
// ---------------------------------------------------------------------------
namespace {

gplan::Point P( const VECTOR2I& v ) { return { (double) v.x, (double) v.y }; }

// Axis-aligned bounding box of a board item as a convex gplan polygon.
gplan::Polygon bboxPoly( const BOX2I& b )
{
    return { { (double) b.GetLeft(),  (double) b.GetTop() },
             { (double) b.GetRight(), (double) b.GetTop() },
             { (double) b.GetRight(), (double) b.GetBottom() },
             { (double) b.GetLeft(),  (double) b.GetBottom() } };
}

// Exact swept rectangle of a track segment (movable, any shape is fine).
gplan::Polygon trackPoly( const VECTOR2I& a, const VECTOR2I& b, int width )
{
    VECTOR2D d( b.x - a.x, b.y - a.y );
    double l = std::hypot( d.x, d.y );
    if( l < 1 ) l = 1;
    VECTOR2D n( -d.y / l * width / 2.0, d.x / l * width / 2.0 );
    return { { a.x + n.x, a.y + n.y }, { b.x + n.x, b.y + n.y },
             { b.x - n.x, b.y - n.y }, { a.x - n.x, a.y - n.y } };
}

} // namespace

std::vector<gplan::Obstacle> PnsBridge::getObstacles( int pnsLayer ) const
{
    std::vector<gplan::Obstacle> out;

    auto onLayer = [&]( BOARD_ITEM* it, PCB_LAYER_ID bl )
    { return this->pnsLayer( bl ) == pnsLayer && it->IsOnLayer( bl ); };

    // Tracks / arcs / vias.
    for( PCB_TRACK* t : m_board->Tracks() )
    {
        if( t->Type() == PCB_VIA_T )
        {
            PCB_VIA* v = static_cast<PCB_VIA*>( t );
            if( this->pnsLayer( v->GetLayer() ) <= pnsLayer
                && this->pnsLayer( v->BottomLayer() ) >= pnsLayer )
                out.push_back( { bboxPoly( v->GetBoundingBox() ), v->IsLocked(), pnsLayer } );
        }
        else if( onLayer( t, t->GetLayer() ) )
        {
            // movable copper -> SOFT (unless locked)
            out.push_back( { trackPoly( t->GetStart(), t->GetEnd(), t->GetWidth() ),
                             t->IsLocked(), pnsLayer } );
        }
    }

    // Pads -> FIXED.
    for( FOOTPRINT* fp : m_board->Footprints() )
        for( PAD* pad : fp->Pads() )
            for( PCB_LAYER_ID bl : pad->GetLayerSet().CuStack() )
                if( onLayer( pad, bl ) )
                {
                    out.push_back( { bboxPoly( pad->GetBoundingBox() ), true, pnsLayer } );
                    break;
                }

    // Keepout / rule-area zones -> FIXED.
    for( ZONE* z : m_board->Zones() )
        if( z->GetIsRuleArea() )
            for( PCB_LAYER_ID bl : z->GetLayerSet().CuStack() )
                if( onLayer( z, bl ) )
                {
                    out.push_back( { bboxPoly( z->GetBoundingBox() ), true, pnsLayer } );
                    break;
                }

    return out;
}

std::vector<gplan::Obstacle> PnsBridge::getAllObstacles() const
{
    std::vector<gplan::Obstacle> all;
    for( PCB_LAYER_ID bl : m_board->GetEnabledLayers().CuStack() )
    {
        int pl = pnsLayer( bl );
        for( gplan::Obstacle& o : getObstacles( pl ) )
            all.push_back( std::move( o ) );
    }
    return all;
}

// ---------------------------------------------------------------------------
// Route + verify (gplan waypoints -> PNS shove -> outcome)
// ---------------------------------------------------------------------------
RouteResult PnsBridge::routeAndCheck( const std::vector<gplan::Waypoint>& wps )
{
    RouteResult r;
    if( wps.size() < 2 )
        return r;

    auto toV = []( const gplan::Waypoint& w )
    { return VECTOR2I( (int) std::lround( w.p.x ), (int) std::lround( w.p.y ) ); };

    VECTOR2I startP = toV( wps.front() );
    int      layer  = wps.front().layer;

    // Pick the PNS item under the start point (a pad/track on the target net).
    // Try an exact hit first, then with a small slop radius to catch pad edges.
    PNS::ITEM_SET startHits = m_router->QueryHoverItems( startP );
    if( startHits.Empty() )
        startHits = m_router->QueryHoverItems( startP, 100000 ); // ~0.1 mm
    PNS::ITEM*    startItem = startHits.Empty() ? nullptr : startHits[0];

    // Import track/via sizes from the start item + net rules.
    PNS::SIZES_SETTINGS sizes( m_router->Sizes() );
    m_iface->SetStartLayerFromPNS( layer );
    m_iface->ImportSizes( sizes, startItem, startItem ? startItem->Net() : nullptr,
                          VECTOR2D( startP.x, startP.y ) );
    m_router->UpdateSizes( sizes );

    if( !m_router->StartRouting( startP, startItem, layer ) )
    {
        r.reason = m_router->FailureReason().ToStdString();
        return r;
    }

    // Walk the intermediate waypoints. A layer change == drop a via there.
    // A via only commits on a FixRoute (forceCommit=false keeps it speculative —
    // it lands in the session node, not the board), after which the head
    // continues on the new layer.
    for( size_t i = 1; i < wps.size(); ++i )
    {
        VECTOR2I p = toV( wps[i] );

        if( wps[i].layer != wps[i - 1].layer )
        {
            m_router->Move( p, nullptr );                  // bring head to via site
            if( !m_router->IsPlacingVia() )
                m_router->ToggleViaPlacement();            // arm a via on the head
            m_router->SwitchLayer( wps[i].layer );         // far side of the via
            m_router->FixRoute( p, nullptr, false, false ); // commit seg + via
            if( m_router->IsPlacingVia() )
                m_router->ToggleViaPlacement();            // disarm
            continue;                                       // head now on new layer
        }

        m_router->Move( p, nullptr );
    }

    // --- Evaluate (mirror of ROUTER::markViolations) -----------------------
    // The "head" trace exists after Move() even without a FixRoute commit, so
    // base the verdict on Traces() rather than HasPlacedAnything() (which is
    // only true once a segment has been fixed/committed).
    PNS::PLACEMENT_ALGO* placer = m_router->Placer();
    PNS::NODE*     node   = placer ? placer->CurrentNode( true ) : nullptr;
    PNS::ITEM_SET  traces = placer ? placer->Traces() : PNS::ITEM_SET();
    r.placed = node && traces.Size() > 0;
    if( !r.placed )
    {
        r.reason = m_router->FailureReason().ToStdString();
        m_router->StopRouting();
        return r;
    }

    for( PNS::ITEM* item : traces.Items() )
    {
        PNS::NODE::OBSTACLES obs;
        node->QueryColliding( item, obs );
        if( item->OfKind( PNS::ITEM::LINE_T ) )
        {
            PNS::LINE* l = static_cast<PNS::LINE*>( item );
            if( l->EndsWithVia() )
            {
                PNS::VIA v( l->Via() );
                node->QueryColliding( &v, obs );
            }
        }
        if( !obs.empty() )
        {
            r.collided = true;
            r.blocking = P( obs.begin()->m_ipFirst );
            break;
        }
    }

    // Reached the target? (head endpoint near the last waypoint)
    PNS::LINE* head = traces.Size() ? static_cast<PNS::LINE*>( traces[0] ) : nullptr;
    bool reached = head && head->PointCount()
                   && ( head->CLine().CPoint( -1 ) - toV( wps.back() ) ).EuclideanNorm() < 1000;

    // Count vias the route placed (proves the layer-change / via path ran).
    PNS::NODE::ITEM_VECTOR removedItems, addedItems;
    node->GetUpdatedItems( removedItems, addedItems );
    for( PNS::ITEM* it : addedItems )
        if( it->OfKind( PNS::ITEM::VIA_T ) )
            r.vias++;

    r.ok = r.placed && reached && !r.collided;
    r.reason = m_router->FailureReason().ToStdString();

    // Discard everything — this is a speculative trial, commit nothing.
    m_router->StopRouting();
    return r;
}

// ---------------------------------------------------------------------------
// Route + extract geometry (same shove as routeAndCheck, but returns the
// resulting segments + vias so the host can apply them to a board).
// ---------------------------------------------------------------------------
RouteGeom PnsBridge::routeAndExtract( const std::vector<gplan::Waypoint>& wps )
{
    RouteGeom g;
    if( wps.size() < 2 )
        return g;

    auto toV = []( const gplan::Waypoint& w )
    { return VECTOR2I( (int) std::lround( w.p.x ), (int) std::lround( w.p.y ) ); };

    VECTOR2I startP = toV( wps.front() );
    int      layer  = wps.front().layer;

    PNS::ITEM_SET startHits = m_router->QueryHoverItems( startP );
    if( startHits.Empty() )
        startHits = m_router->QueryHoverItems( startP, 100000 );
    PNS::ITEM* startItem = startHits.Empty() ? nullptr : startHits[0];

    PNS::SIZES_SETTINGS sizes( m_router->Sizes() );
    m_iface->SetStartLayerFromPNS( layer );
    m_iface->ImportSizes( sizes, startItem, startItem ? startItem->Net() : nullptr,
                          VECTOR2D( startP.x, startP.y ) );
    m_router->UpdateSizes( sizes );

    if( startItem )
        g.netcode = m_iface->GetNetCode( startItem->Net() );

    if( !m_router->StartRouting( startP, startItem, layer ) )
    {
        g.reason = m_router->FailureReason().ToStdString();
        return g;
    }

    for( size_t i = 1; i < wps.size(); ++i )
    {
        VECTOR2I p = toV( wps[i] );
        if( wps[i].layer != wps[i - 1].layer )
        {
            m_router->Move( p, nullptr );
            if( !m_router->IsPlacingVia() )
                m_router->ToggleViaPlacement();
            m_router->SwitchLayer( wps[i].layer );
            m_router->FixRoute( p, nullptr, false, false );
            if( m_router->IsPlacingVia() )
                m_router->ToggleViaPlacement();
            continue;
        }
        m_router->Move( p, nullptr );
    }

    // Seal the remaining head segments into the session node (forceFinish=true,
    // forceCommit=false → lands in the node, NOT the board) so the full route is
    // enumerable below.
    m_router->FixRoute( toV( wps.back() ), nullptr, true, false );

    PNS::PLACEMENT_ALGO* placer = m_router->Placer();
    PNS::NODE*           node   = placer ? placer->CurrentNode( true ) : nullptr;
    if( !node )
    {
        g.reason = m_router->FailureReason().ToStdString();
        m_router->StopRouting();
        return g;
    }

    PNS::NODE::ITEM_VECTOR removedItems, addedItems;
    node->GetUpdatedItems( removedItems, addedItems );

    for( PNS::ITEM* it : addedItems )
    {
        if( it->OfKind( PNS::ITEM::SEGMENT_T ) )
        {
            auto* s = static_cast<PNS::SEGMENT*>( it );
            const SEG& sg = s->Seg();
            int bl = m_iface->GetBoardLayerFromPNSLayer( s->Layer() );
            g.segs.push_back( { (double) sg.A.x, (double) sg.A.y,
                                (double) sg.B.x, (double) sg.B.y,
                                (double) s->Width(), (double) bl } );
            g.segNets.push_back( m_iface->GetNetName( s->Net() ).ToStdString() );
        }
        else if( it->OfKind( PNS::ITEM::VIA_T ) )
        {
            auto* v = static_cast<PNS::VIA*>( it );
            int top = m_iface->GetBoardLayerFromPNSLayer( v->Layers().Start() );
            int bot = m_iface->GetBoardLayerFromPNSLayer( v->Layers().End() );
            g.viaList.push_back( { (double) v->Pos().x, (double) v->Pos().y,
                                   (double) v->Diameter( v->Layers().Start() ),
                                   (double) v->Drill(), (double) top, (double) bot } );
            g.viaNets.push_back( m_iface->GetNetName( v->Net() ).ToStdString() );
            g.vias++;
        }
    }

    // REMOVED items: shoved neighbours at their old positions. The host deletes
    // these so the shove is realized instead of duplicated.
    for( PNS::ITEM* it : removedItems )
    {
        if( it->OfKind( PNS::ITEM::SEGMENT_T ) )
        {
            auto* s = static_cast<PNS::SEGMENT*>( it );
            const SEG& sg = s->Seg();
            int bl = m_iface->GetBoardLayerFromPNSLayer( s->Layer() );
            g.removedSegs.push_back( { (double) sg.A.x, (double) sg.A.y,
                                       (double) sg.B.x, (double) sg.B.y,
                                       (double) s->Width(), (double) bl } );
        }
        else if( it->OfKind( PNS::ITEM::VIA_T ) )
        {
            auto* v = static_cast<PNS::VIA*>( it );
            int top = m_iface->GetBoardLayerFromPNSLayer( v->Layers().Start() );
            int bot = m_iface->GetBoardLayerFromPNSLayer( v->Layers().End() );
            g.removedVias.push_back( { (double) v->Pos().x, (double) v->Pos().y,
                                       (double) top, (double) bot } );
        }
    }

    g.placed = !g.segs.empty();

    // Residual collisions against the sealed node (same check as routeAndCheck).
    for( PNS::ITEM* it : addedItems )
    {
        PNS::NODE::OBSTACLES obs;
        node->QueryColliding( it, obs );
        if( !obs.empty() ) { g.collided = true; break; }
    }

    g.ok = g.placed && !g.collided;
    g.reason = m_router->FailureReason().ToStdString();

    // Discard the session — host applies g.segs / g.viaList to the board itself.
    m_router->StopRouting();
    return g;
}
