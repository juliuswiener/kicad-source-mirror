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
        std::printf( "routeAndCheck: ok=%d placed=%d collided=%d reason='%s'\n",
                     r.ok, r.placed, r.collided, r.reason.c_str() );
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
