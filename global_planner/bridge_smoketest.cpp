// bridge_smoketest.cpp
//
// Real link + runtime test of the KiCad-linked bridge. Loads a board with the
// minimal, kiface-free recipe (PCB_IO_KICAD_SEXPR + a hand-built DRC_ENGINE, as
// qa/tools/pns/pns_log_file.cpp does), attaches the bridge, extracts obstacles
// from the live PNS world, runs the gplan core over them, and exercises
// routeAndCheck. Proves the bridge links and runs against real KiCad/PNS.
//
// Built only when KICAD_BUILD_PNS_DEBUG_TOOL=ON (see qa/tools/pns/CMakeLists.txt).

#include <wx/init.h>
#include <wx/filename.h>
#include <cstdio>
#include <memory>
#include <map>
#include <vector>

#include <pad.h>
#include <footprint.h>
#include <pcb_track.h>

#include <pgm_base.h>
#include <board.h>
#include <board_design_settings.h>
#include <project.h>
#include <settings/settings_manager.h>
#include <pcbnew/pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <pcbnew/drc/drc_engine.h>
#include <wildcards_and_files_ext.h>
#include <layer_ids.h>

#include "pns_bridge.h"
#include "planner_core.h"

// Minimal concrete PGM_BASE so Pgm() is valid in this standalone harness.
struct SMOKE_PGM : public PGM_BASE
{
    void MacOpenFile( const wxString& ) override {}
};
static SMOKE_PGM s_pgm;

static int run( int argc, char** argv )
{
    wxFileName fnPcb( ( argc > 1 ) ? wxString::FromUTF8( argv[1] )
                                   : wxString( "qa/data/pcbnew/complex_hierarchy.kicad_pcb" ) );
    fnPcb.MakeAbsolute();
    const wxString pcb = fnPcb.GetFullPath();

    // --- minimal, kiface-free board load (mirrors pns_log_file.cpp) ---------
    SETTINGS_MANAGER settingsMgr;
    wxFileName fnPro( pcb );
    fnPro.SetExt( wxT( "kicad_pro" ) );
    settingsMgr.LoadProject( fnPro.GetFullPath() );
    PROJECT* project = settingsMgr.GetProject( fnPro.GetFullPath() );

    std::unique_ptr<BOARD> board;
    try
    {
        PCB_IO_KICAD_SEXPR io;
        board.reset( io.LoadBoard( pcb, nullptr, nullptr ) );
    }
    catch( ... )
    {
        std::printf( "FAIL: could not load board '%s'\n", (const char*) pcb.utf8_str() );
        return 1;
    }
    if( !board )
    {
        std::printf( "FAIL: null board\n" );
        return 1;
    }

    board->SetProject( project );

    auto drcEngine = std::make_shared<DRC_ENGINE>();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    bds.m_DRCEngine = drcEngine;
    board->SynchronizeNetsAndNetClasses( true );
    drcEngine->SetBoard( board.get() );
    drcEngine->SetDesignSettings( &bds );

    wxFileName fnRules( pcb );
    fnRules.SetExt( FILEEXT::DesignRulesFileExtension );
    drcEngine->InitEngine( fnRules.FileExists() ? fnRules : wxFileName() );

    std::printf( "board loaded: %zu tracks, %zu footprints\n",
                 board->Tracks().size(), board->Footprints().size() );

    // --- bridge ------------------------------------------------------------
    gbridge::PnsBridge br;
    if( !br.attach( board.get() ) )
    {
        std::printf( "FAIL: bridge attach\n" );
        return 1;
    }
    std::printf( "bridge attached, PNS world synced\n" );

    std::vector<gplan::Obstacle> obs = br.getAllObstacles();
    size_t fixed = 0, movable = 0;
    for( const auto& o : obs ) ( o.fixed ? fixed : movable )++;
    std::printf( "obstacles extracted: %zu (fixed=%zu, movable=%zu)\n",
                 obs.size(), fixed, movable );
    if( obs.empty() )
    {
        std::printf( "FAIL: no obstacles extracted from a populated board\n" );
        return 1;
    }

    // --- core planner on F.Cu ---------------------------------------------
    gplan::PlannerParams p;
    p.trackWidth = 200000;   // 0.2 mm in nm
    p.clearance  = 200000;
    p.kPaths     = 3;
    int fcu = br.pnsLayer( F_Cu );
    std::vector<gplan::Obstacle> fcuObs;
    for( const auto& o : obs ) if( o.layer == fcu ) fcuObs.push_back( o );
    std::printf( "F.Cu (pns layer %d) obstacles: %zu\n", fcu, fcuObs.size() );

    gplan::Planner planner( fcuObs, p );

    // Pick a real net with >=2 pads on F.Cu and route between two of its pads,
    // so PNS actually has start/target items to work with.
    std::map<int, std::vector<VECTOR2I>> netPads;
    for( FOOTPRINT* fp : board->Footprints() )
        for( PAD* pad : fp->Pads() )
            if( pad->GetNetCode() > 0 && pad->IsOnLayer( F_Cu ) )
                netPads[pad->GetNetCode()].push_back( pad->GetPosition() );

    VECTOR2I pa, pb;
    bool havePair = false;
    for( auto& [net, pts] : netPads )
        if( pts.size() >= 2 )
        {
            pa = pts.front();
            pb = pts.back();
            havePair = true;
            std::printf( "routing net %d between (%d,%d) and (%d,%d)\n",
                         net, pa.x, pa.y, pb.x, pb.y );
            break;
        }
    if( !havePair )
    {
        std::printf( "no multi-pad F.Cu net found; skipping route\n" );
        std::printf( "SMOKETEST OK\n" );
        return 0;
    }

    gplan::Point a{ (double) pa.x, (double) pa.y };
    gplan::Point b{ (double) pb.x, (double) pb.y };
    auto paths = planner.plan( a, b );
    std::printf( "gplan candidates for the net: %zu\n", paths.size() );

    // --- exercise PNS route+verify path (must run, verdict not asserted) ----
    if( !paths.empty() )
    {
        gbridge::RouteResult r = br.routeAndCheck( paths.front().waypoints );
        std::printf( "routeAndCheck: ok=%d placed=%d vias=%d collided=%d reason='%s'\n",
                     r.ok, r.placed, r.vias, r.collided, r.reason.c_str() );
    }

    // --- forced multi-layer route: exercise the via path deterministically --
    int bcu = br.pnsLayer( B_Cu );
    VECTOR2I mid( ( pa.x + pb.x ) / 2, ( pa.y + pb.y ) / 2 );
    std::vector<gplan::Waypoint> ml = {
        { { (double) pa.x,  (double) pa.y  }, fcu },
        { { (double) mid.x, (double) mid.y }, fcu },
        { { (double) mid.x, (double) mid.y }, bcu },   // <- via here
        { { (double) pb.x,  (double) pb.y  }, bcu },
    };
    std::printf( "multilayer route F.Cu(%d)->B.Cu(%d), via at (%d,%d)\n",
                 fcu, bcu, mid.x, mid.y );
    gbridge::RouteResult mr = br.routeAndCheck( ml );
    std::printf( "multilayer routeAndCheck: ok=%d placed=%d vias=%d collided=%d reason='%s'\n",
                 mr.ok, mr.placed, mr.vias, mr.collided, mr.reason.c_str() );
    if( mr.placed && mr.vias >= 1 )
        std::printf( "VIA PATH OK (%d via placed)\n", mr.vias );
    else
        std::printf( "VIA PATH: no via placed (placed=%d) — see notes\n", mr.placed );

    // --- routeAndExtract: get the routed geometry back for the host to apply ---
    if( !paths.empty() )
    {
        gbridge::RouteGeom g = br.routeAndExtract( paths.front().waypoints );
        std::printf( "routeAndExtract: ok=%d placed=%d net=%d segs=%zu vias=%zu "
                     "removedSegs=%zu removedVias=%zu reason='%s'\n",
                     g.ok, g.placed, g.netcode, g.segs.size(), g.viaList.size(),
                     g.removedSegs.size(), g.removedVias.size(), g.reason.c_str() );
        if( !g.segs.empty() )
        {
            const auto& s = g.segs.front();   // {x1,y1,x2,y2,width,boardLayer}
            std::printf( "  first seg: (%.0f,%.0f)->(%.0f,%.0f) w=%.0f layer=%.0f\n",
                         s[0], s[1], s[2], s[3], s[4], s[5] );
        }
        if( g.placed && !g.segs.empty() )
            std::printf( "EXTRACT OK (%zu segs, %zu vias returned)\n",
                         g.segs.size(), g.viaList.size() );
        else
            std::printf( "EXTRACT: nothing returned (placed=%d)\n", g.placed );
    }

    // --- T12: route the same net under Walkaround as well as Shove ----------
    if( !paths.empty() )
    {
        br.setMode( gbridge::PnsBridge::RouteMode::Walkaround );
        gbridge::RouteResult wr = br.routeAndCheck( paths.front().waypoints );
        std::printf( "T12 walkaround routeAndCheck: ok=%d placed=%d collided=%d reason='%s'\n",
                     wr.ok, wr.placed, wr.collided, wr.reason.c_str() );
        br.setMode( gbridge::PnsBridge::RouteMode::Shove );   // restore default
        if( !wr.placed )
        { std::printf( "T12 FAIL: walkaround placed nothing\n" ); return 1; }
    }

    // --- T10: cleanup + re-attach to the same board + route again -----------
    if( !paths.empty() )
    {
        br.cleanup();
        if( !br.attach( board.get() ) )
        { std::printf( "T10 FAIL: re-attach\n" ); return 1; }
        gbridge::RouteResult rr = br.routeAndCheck( paths.front().waypoints );
        std::printf( "T10 re-attach routeAndCheck: ok=%d placed=%d\n", rr.ok, rr.placed );
        if( !rr.placed )
        { std::printf( "T10 FAIL: route after re-attach placed nothing\n" ); return 1; }
    }

    // --- routeAndCommit: lossless commit-to-world (PNS change stream) --------
    if( !paths.empty() )
    {
        gbridge::RouteChange c = br.routeAndCommit( paths.front().waypoints );
        std::printf( "routeAndCommit: ok=%d placed=%d reached=%d collided=%d net=%d "
                     "added=%zu mod=%zu removed=%zu vias=%d blocking=(%.0f,%.0f)\n",
                     c.ok, c.placed, c.reached, c.collided, c.netcode, c.addedSegs.size(),
                     c.modSegUuids.size(), c.removedUuids.size(), c.vias,
                     c.blocking.x, c.blocking.y );
        if( !c.ok )
        { std::printf( "COMMIT FAIL: did not reach target (honest)\n" ); return 1; }
        std::printf( "COMMIT OK (reached, %zu added segs, %zu modified neighbours by uuid)\n",
                     c.addedSegs.size(), c.modSegUuids.size() );
    }

    // --- probeTarget: seedable / congested flag for a target ----------------
    {
        int    bcu = br.pnsLayer( B_Cu );
        double clr = 200000;
        gbridge::TargetProbe t1 = br.probeTarget( pa.x, pa.y, fcu, 1, clr, bcu );
        std::printf( "probeTarget net-1 pad: seedable=%d congested=%d nearestForeign=%.0f "
                     "foreignNet=%d nearestOther=%.0f\n",
                     t1.seedable, t1.congested, t1.nearestForeign, t1.foreignNet, t1.nearestOther );
        gbridge::TargetProbe t2 = br.probeTarget( pa.x + 50000000, pa.y, fcu, 1, clr, bcu );
        std::printf( "probeTarget open point: seedable=%d congested=%d\n",
                     t2.seedable, t2.congested );
        if( t2.seedable )
        { std::printf( "PROBE FAIL: open point reported seedable\n" ); return 1; }
        std::printf( "PROBE OK\n" );
    }

    // --- T9: make net 1 unrouted (remove its tracks), find the ratsnest target
    //          via the bridge, and route the now-unrouted net. ----------------
    {
        std::vector<PCB_TRACK*> rm;
        for( PCB_TRACK* t : board->Tracks() )
            if( t->GetNetCode() == 1 && t->Type() != PCB_VIA_T )
                rm.push_back( t );
        for( PCB_TRACK* t : rm )
            board->Remove( t );
        board->BuildConnectivity();
        br.cleanup();
        br.attach( board.get() );
        std::printf( "T9: removed %zu net-1 tracks, re-synced\n", rm.size() );

        VECTOR2I padP; bool haveStart = false;
        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
                if( pad->GetNetCode() == 1 && pad->IsOnLayer( F_Cu ) )
                { padP = pad->GetPosition(); haveStart = true; break; }
            if( haveStart ) break;
        }

        if( haveStart )
        {
            auto tgt = br.nearestUnconnected( padP.x, padP.y, fcu );
            if( !tgt )
            { std::printf( "T9 FAIL: no ratsnest target on the unrouted net\n" ); return 1; }
            std::printf( "T9 ratsnest target: (%.0f,%.0f) L%d\n",
                         tgt->p.x, tgt->p.y, tgt->layer );
            std::vector<gplan::Waypoint> wp = {
                { { (double) padP.x, (double) padP.y }, fcu }, *tgt };
            gbridge::RouteResult r = br.routeAndCheck( wp );
            std::printf( "T9 routeAndCheck on unrouted net: ok=%d placed=%d collided=%d\n",
                         r.ok, r.placed, r.collided );
            if( !r.placed )
            { std::printf( "T9 FAIL: could not route the now-unrouted net\n" ); return 1; }
        }
        else
            std::printf( "T9: no net-1 F.Cu pad found; skipping\n" );
    }

    std::printf( "SMOKETEST OK\n" );
    return 0;
}

int main( int argc, char** argv )
{
    wxInitialize( argc, argv );
    wxDisableAsserts();   // headless: never block on a wx assert dialog
    SetPgm( &s_pgm );
    Pgm().InitPgm( /*aHeadless=*/ true, /*aIsUnitTest=*/ true );

    int rc = run( argc, argv );
    std::fflush( stdout );

    // KiCad's global/static teardown (Pgm, library-manager async loads, wx) is
    // unreliable in a bare headless harness and can hang/crash at exit. The work
    // is done and flushed, so hard-exit with the real status instead.
    std::_Exit( rc );
}
