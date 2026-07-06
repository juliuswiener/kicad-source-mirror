// zone_refill_tool.cpp
//
// Standalone headless CLI: load a board, optionally insert a candidate via,
// run the REAL KiCad zone filler (carves antipads around every via/pad that
// exists at fill time), save the board back out.
//
// WHY A SEPARATE BINARY: ZONE_FILLER::Fill()'s real implementation only lives
// in pcbnew_kiface_objects (the object library baked into the pcbnew GUI
// kiface module) -- it is not available as a lightweight standalone lib. The
// gplan bridge (pns_bridge.cpp) deliberately avoids linking pcbnew_kiface /
// its QA mocks (mocks.cpp) for a fast, dependency-light headless build; a
// full ZONE_FILLER pulls in the entire kiface object set, which conflicts
// with those mocks symbol-for-symbol. Kept as its own process: run this tool
// BEFORE a gplan/PNS session that needs to place a via crossing a plane zone
// (the classic chicken-egg: a new via has no antipad until some fill runs
// with that via already present), then re-load the refilled board in gplan.
//
// Usage:
//   gplan_zone_refill <board.kicad_pcb>
//   gplan_zone_refill <board.kicad_pcb> --add-via x y drill dia net top bottom
//     x,y,drill,dia in board units (nm); net = net name; top/bottom = KiCad
//     copper layer names ("F.Cu", "In2.Cu", "B.Cu", ...).
//
// Exit 0 on success (prints "ZONES REFILLED: <n>"), 1 on any failure.

#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <wx/init.h>
#include <wx/filename.h>

#include <pgm_base.h>
#include <kiface_base.h>
#include <board.h>
#include <board_design_settings.h>
#include <netinfo.h>
#include <pcb_track.h>
#include <project.h>
#include <settings/settings_manager.h>
#include <pcbnew/pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <pcbnew/drc/drc_engine.h>
#include <pcbnew/zone_filler.h>
#include <wildcards_and_files_ext.h>
#include <zone.h>

struct REFILL_PGM : public PGM_BASE
{
    void MacOpenFile( const wxString& ) override {}
};
static REFILL_PGM s_pgm;

// pcbnew_kiface_objects (needed for the real ZONE_FILLER::Fill()) references
// the global Kiface() accessor throughout -- normally defined in pcbnew.cpp
// alongside a full GUI-capable KIFACE_BASE. We never touch windowing/kiway
// messaging here, so a minimal stub with no-op pure virtuals is sufficient.
namespace {
struct MINIMAL_KIFACE : public KIFACE_BASE
{
    MINIMAL_KIFACE() : KIFACE_BASE( "pcbnew", KIWAY::FACE_PCB ) {}
    bool OnKifaceStart( PGM_BASE*, int, KIWAY* ) override { return true; }
    wxWindow* CreateKiWindow( wxWindow*, int, KIWAY*, int ) override { return nullptr; }
    void* IfaceOrAddress( int ) override { return nullptr; }
};
MINIMAL_KIFACE s_kiface;
} // namespace

KIFACE_BASE& Kiface()
{
    return s_kiface;
}

static int run( int argc, char** argv )
{
    if( argc < 2 )
    {
        std::fprintf( stderr, "usage: %s <board.kicad_pcb> "
                      "[--add-via x y drill dia net top bottom]\n", argv[0] );
        return 1;
    }

    wxFileName fnPcb( wxString::FromUTF8( argv[1] ) );
    fnPcb.MakeAbsolute();
    const wxString pcb = fnPcb.GetFullPath();

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
    catch( const std::exception& e )
    {
        std::fprintf( stderr, "FAIL: board parse failed: %s\n", e.what() );
        return 1;
    }
    if( !board )
    {
        std::fprintf( stderr, "FAIL: board parse failed: LoadBoard returned null\n" );
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

    if( argc >= 3 && std::string( argv[2] ) == "--add-via" )
    {
        if( argc < 10 )
        {
            std::fprintf( stderr, "FAIL: --add-via needs x y drill dia net top bottom\n" );
            return 1;
        }
        long long x = std::atoll( argv[3] ), y = std::atoll( argv[4] );
        int drill = std::atoi( argv[5] ), dia = std::atoi( argv[6] );
        wxString netName = wxString::FromUTF8( argv[7] );
        wxString topName = wxString::FromUTF8( argv[8] ), botName = wxString::FromUTF8( argv[9] );

        NETINFO_ITEM* net = board->GetNetInfo().GetNetItem( netName );
        if( !net )
        {
            std::fprintf( stderr, "FAIL: unknown net '%s'\n", (const char*) netName.utf8_str() );
            return 1;
        }

        PCB_LAYER_ID top = board->GetLayerID( topName ), bot = board->GetLayerID( botName );
        if( top == UNDEFINED_LAYER || bot == UNDEFINED_LAYER )
        {
            std::fprintf( stderr, "FAIL: unknown layer '%s' or '%s'\n",
                         (const char*) topName.utf8_str(), (const char*) botName.utf8_str() );
            return 1;
        }

        std::unique_ptr<PCB_VIA> via = std::make_unique<PCB_VIA>( board.get() );
        via->SetPosition( VECTOR2I( x, y ) );
        via->SetDrill( drill );
        via->SetWidth( dia );
        via->SetLayerPair( top, bot );
        via->SetNet( net );

        // The ctor defaults to VIATYPE::THROUGH regardless of the layer span
        // set above; PNS (pns_kicad_iface.cpp) branches on GetViaType() ==
        // BLIND/BURIED to decide per-layer behaviour, so a THROUGH-flagged
        // via with a partial span is treated as an inconsistent/invalid
        // through via. Derive the real type from the span instead (order-
        // independent: caller may pass top/bottom in either order).
        bool spansFullStack = ( top == F_Cu && bot == B_Cu ) || ( top == B_Cu && bot == F_Cu );
        bool touchesOuter = top == F_Cu || top == B_Cu || bot == F_Cu || bot == B_Cu;
        if( !spansFullStack )
            via->SetViaType( touchesOuter ? VIATYPE::BLIND : VIATYPE::BURIED );
        board->Add( via.release(), ADD_MODE::INSERT );
        std::printf( "candidate via added at (%lld,%lld) net=%s %s->%s\n",
                     x, y, (const char*) netName.utf8_str(),
                     (const char*) topName.utf8_str(), (const char*) botName.utf8_str() );
    }

    // No explicit board->BuildConnectivity() -- ZONE_FILLER::Fill() does its own
    // internally, and calling it early reassigned a clearance-less new via's net
    // to the zone's net (the exact chicken-egg this tool exists to break).

    ZONE_FILLER filler( board.get(), /*aCommit=*/ nullptr );
    bool ok = filler.Fill( board->Zones() );
    if( !ok )
    {
        std::fprintf( stderr, "FAIL: ZONE_FILLER::Fill() reported failure/cancel\n" );
        return 1;
    }

    try
    {
        PCB_IO_KICAD_SEXPR io;
        io.SaveBoard( pcb, board.get(), nullptr );
    }
    catch( const std::exception& e )
    {
        std::fprintf( stderr, "FAIL: SaveBoard failed: %s\n", e.what() );
        return 1;
    }

    std::printf( "ZONES REFILLED: %zu\n", board->Zones().size() );
    return 0;
}

int main( int argc, char** argv )
{
    wxInitialize( argc, argv );
    wxDisableAsserts();
    SetPgm( &s_pgm );
    Pgm().InitPgm( /*aHeadless=*/ true, /*aIsUnitTest=*/ true );

    int rc = run( argc, argv );
    std::fflush( stdout );
    std::_Exit( rc );   // headless KiCad static teardown is unreliable (see bridge_smoketest.cpp)
}
