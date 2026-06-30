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

bool PnsBridge::load( const std::string& pcbPath )
{
    const wxString pcb = wxString::FromUTF8( pcbPath );

    // Project (.kicad_pro): netclasses + design rules, by the usual stem.
    m_settings = std::make_unique<SETTINGS_MANAGER>();
    wxFileName fnPro( pcb );
    fnPro.SetExt( wxT( "kicad_pro" ) );
    m_settings->LoadProject( fnPro.GetFullPath() );
    PROJECT* project = m_settings->GetProject( fnPro.GetFullPath() );

    std::unique_ptr<BOARD> board;
    try
    {
        PCB_IO_KICAD_SEXPR io;
        board.reset( io.LoadBoard( pcb, nullptr, nullptr ) );
    }
    catch( ... )
    {
        return false;
    }
    if( !board )
        return false;

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
    return attach( m_boardHolder.get() );
}
