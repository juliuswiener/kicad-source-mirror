// pns_bridge_load.cpp
//
// The convenience board loader for PnsBridge. Uses the kiface-FREE recipe
// (PCB_IO_KICAD_SEXPR + a hand-built DRC_ENGINE, exactly as bridge_smoketest.cpp
// / qa/tools/pns/pns_log_file.cpp do) so the bridge links without BOARD_LOADER
// (the pcbnew kiface). This is the headless-proven path and lets the Python
// module (gplan_kicad) load a board with no kiface dependency.

#include "pns_bridge.h"

#include <board.h>
#include <board_design_settings.h>
#include <project.h>
#include <settings/settings_manager.h>
#include <pcbnew/pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <pcbnew/drc/drc_engine.h>
#include <wildcards_and_files_ext.h>
#include <wx/filename.h>

using namespace gbridge;

bool PnsBridge::load( const std::string& pcbPath, std::string* aErr )
{
    // Resolve to absolute BEFORE anything else: LoadProject() below silently
    // fails on a relative path once cwd has drifted from a prior successful
    // load (LoadProject() itself changes cwd to the project dir as a side
    // effect) — a relative path here would then resolve against the WRONG
    // directory. Fixing at entry makes load() robust regardless of caller cwd.
    wxFileName fnAbs( wxString::FromUTF8( pcbPath ) );
    fnAbs.MakeAbsolute();
    const wxString pcb = fnAbs.GetFullPath();
    auto fail = [aErr]( const std::string& msg ) { if( aErr ) *aErr = msg; return false; };

    // Reload teardown order: the old board's BOARD_DESIGN_SETTINGS is a
    // NESTED_SETTINGS of the OLD project's file. Replacing m_settings (next
    // line) destroys that project, but the old board is only destroyed later
    // (m_boardHolder re-assignment) — its ~BOARD_DESIGN_SETTINGS would then
    // call ReleaseNestedSettings() on the freed parent, an intermittent
    // (heap-layout dependent) use-after-free crash. Detach while the old
    // project is still alive.
    if( m_boardHolder )
        m_boardHolder->ClearProject();

    // Project (.kicad_pro): netclasses + design rules, by the usual stem.
    m_settings = std::make_unique<SETTINGS_MANAGER>();
    wxFileName fnPro( pcb );
    fnPro.SetExt( wxT( "kicad_pro" ) );
    if( !m_settings->LoadProject( fnPro.GetFullPath() ) )
        return fail( "project load failed: " + std::string( fnPro.GetFullPath().utf8_str() ) );
    PROJECT* project = m_settings->GetProject( fnPro.GetFullPath() );

    std::unique_ptr<BOARD> board;
    try
    {
        PCB_IO_KICAD_SEXPR io;
        board.reset( io.LoadBoard( pcb, nullptr, nullptr ) );
    }
    catch( const std::exception& e )
    {
        return fail( "board parse failed: " + std::string( e.what() ) );
    }
    catch( ... )
    {
        return fail( "board parse failed: unknown exception" );
    }
    if( !board )
        return fail( "board parse failed: LoadBoard returned null" );

    board->SetProject( project );

    // Build + init the DRC engine on <board>.kicad_dru so custom rules are live.
    auto drcEngine = std::make_shared<DRC_ENGINE>();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    bds.m_DRCEngine = drcEngine;
    board->SynchronizeNetsAndNetClasses( true );
    drcEngine->SetBoard( board.get() );
    drcEngine->SetDesignSettings( &bds );

    wxFileName fnRules( pcb );
    fnRules.SetExt( FILEEXT::DesignRulesFileExtension );
    drcEngine->InitEngine( fnRules.FileExists() ? fnRules : wxFileName() );

    m_boardHolder = std::move( board );
    if( !attach( m_boardHolder.get() ) )
        return fail( "attach failed" );
    return true;
}

bool PnsBridge::save( const std::string& pcbPath, std::string* aErr )
{
    auto fail = [aErr]( const std::string& msg ) { if( aErr ) *aErr = msg; return false; };

    if( !m_boardHolder )
        return fail( "no board loaded" );

    wxFileName fnAbs( wxString::FromUTF8( pcbPath ) );
    fnAbs.MakeAbsolute();

    try
    {
        PCB_IO_KICAD_SEXPR io;
        io.SaveBoard( fnAbs.GetFullPath(), m_boardHolder.get(), nullptr );
    }
    catch( const std::exception& e )
    {
        return fail( "board save failed: " + std::string( e.what() ) );
    }
    catch( ... )
    {
        return fail( "board save failed: unknown exception" );
    }
    return true;
}
