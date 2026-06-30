// pns_bridge_load.cpp
//
// The convenience board loader for PnsBridge, split into its own translation
// unit because it depends on BOARD_LOADER (compiled into the pcbnew kiface
// objects). Hosts that already have a loaded BOARD link pns_bridge.cpp only and
// call attach(); they do NOT need this file or the kiface.

#include "pns_bridge.h"

#include <board.h>
#include <board_loader.h>
#include <settings/settings_manager.h>
#include <io/io_mgr.h>
#include <wx/filename.h>

using namespace gbridge;

bool PnsBridge::load( const std::string& pcbPath )
{
    m_settings = std::make_unique<SETTINGS_MANAGER>();

    wxFileName pro( wxString::FromUTF8( pcbPath ) );
    pro.SetExt( wxT( "kicad_pro" ) );
    m_settings->LoadProject( pro.GetFullPath() );
    PROJECT* project = &m_settings->Prj();

    // BOARD_LOADER::Load attaches the project, builds the DRC engine and calls
    // InitEngine() on <board>.kicad_dru — so all custom rules are live.
    m_boardHolder = std::shared_ptr<BOARD>(
            BOARD_LOADER::Load( wxString::FromUTF8( pcbPath ),
                                PCB_IO_MGR::KICAD_SEXP, project ).release() );
    if( !m_boardHolder )
        return false;

    return attach( m_boardHolder.get() );
}
