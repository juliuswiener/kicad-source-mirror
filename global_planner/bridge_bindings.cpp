// bridge_bindings.cpp — pybind11 wrapper for the KiCad-linked PNS bridge.
//
// Builds ONLY inside the KiCad tree (links pcbnew/kicommon), alongside
// pns_bridge.cpp + planner_core.cpp. Produces a `gplan_kicad` module so Python
// can run the whole pipeline:
//
//   import gplan, gplan_kicad
//   br = gplan_kicad.PnsBridge(); br.load("board.kicad_pcb")
//   obs = br.get_all_obstacles()                       # -> [gplan.Obstacle]
//   planner = gplan.Planner(obs, params)
//   for path in planner.plan(start, sL, target, tL):
//       r = br.route_and_check(path.waypoints)         # PNS shove + verify
//       if r.ok: break
//       planner.bump_congestion(gplan.Point(r.blocking.x, r.blocking.y), 1.5e6, 5)

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <wx/init.h>
#include <pgm_base.h>

#include "pns_bridge.h"

namespace py = pybind11;
using namespace gbridge;

// Minimal concrete PGM_BASE so Pgm() is valid when the bridge is hosted inside a
// Python process (no KiCad main()). Mirrors the SMOKE_PGM bootstrap that
// bridge_smoketest.cpp does in main(): wx init + headless InitPgm, run once at
// module import.
namespace {
struct GPLAN_PGM : public PGM_BASE
{
    void MacOpenFile( const wxString& ) override {}
};
GPLAN_PGM s_pgm;

void bootstrap_once()
{
    static bool done = false;
    if( done )
        return;
    done = true;

    wxInitialize();
    wxDisableAsserts();        // headless: never block on a wx assert dialog
    SetPgm( &s_pgm );
    Pgm().InitPgm( /*aHeadless=*/ true, /*aIsUnitTest=*/ true );
}
} // namespace

PYBIND11_MODULE( gplan_kicad, m )
{
    bootstrap_once();

    m.doc() = "KiCad-linked PNS bridge for the gplan global planner.";

    py::class_<RouteResult>( m, "RouteResult" )
        .def_readonly( "ok", &RouteResult::ok )
        .def_readonly( "collided", &RouteResult::collided )
        .def_readonly( "placed", &RouteResult::placed )
        .def_readonly( "vias", &RouteResult::vias )
        .def_readonly( "blocking", &RouteResult::blocking )
        .def_readonly( "reason", &RouteResult::reason );

    py::class_<RouteGeom>( m, "RouteGeom" )
        .def_readonly( "ok", &RouteGeom::ok )
        .def_readonly( "collided", &RouteGeom::collided )
        .def_readonly( "placed", &RouteGeom::placed )
        .def_readonly( "vias", &RouteGeom::vias )
        .def_readonly( "netcode", &RouteGeom::netcode )
        .def_readonly( "reason", &RouteGeom::reason )
        .def_readonly( "segs", &RouteGeom::segs )
        .def_readonly( "seg_nets", &RouteGeom::segNets )
        .def_readonly( "via_list", &RouteGeom::viaList )
        .def_readonly( "via_nets", &RouteGeom::viaNets )
        .def_readonly( "removed_segs", &RouteGeom::removedSegs )
        .def_readonly( "removed_vias", &RouteGeom::removedVias );

    py::class_<RouteChange>( m, "RouteChange" )
        .def_readonly( "ok", &RouteChange::ok )
        .def_readonly( "placed", &RouteChange::placed )
        .def_readonly( "reached", &RouteChange::reached )
        .def_readonly( "blocking", &RouteChange::blocking )
        .def_readonly( "collided", &RouteChange::collided )
        .def_readonly( "vias", &RouteChange::vias )
        .def_readonly( "netcode", &RouteChange::netcode )
        .def_readonly( "reason", &RouteChange::reason )
        .def_readonly( "added_segs", &RouteChange::addedSegs )
        .def_readonly( "added_vias", &RouteChange::addedVias )
        .def_readonly( "mod_seg_uuids", &RouteChange::modSegUuids )
        .def_readonly( "mod_segs", &RouteChange::modSegs )
        .def_readonly( "mod_via_uuids", &RouteChange::modViaUuids )
        .def_readonly( "mod_vias", &RouteChange::modVias )
        .def_readonly( "removed_uuids", &RouteChange::removedUuids );

    py::class_<TargetProbe>( m, "TargetProbe" )
        .def_readonly( "seedable", &TargetProbe::seedable )
        .def_readonly( "congested", &TargetProbe::congested )
        .def_readonly( "nearest_foreign", &TargetProbe::nearestForeign )
        .def_readonly( "foreign_net", &TargetProbe::foreignNet )
        .def_readonly( "nearest_other", &TargetProbe::nearestOther );

    auto bridge = py::class_<PnsBridge>( m, "PnsBridge" );
    py::enum_<PnsBridge::RouteMode>( bridge, "RouteMode" )   // T12
        .value( "MARK_OBSTACLES", PnsBridge::RouteMode::MarkObstacles )
        .value( "SHOVE",          PnsBridge::RouteMode::Shove )
        .value( "WALKAROUND",     PnsBridge::RouteMode::Walkaround );
    bridge
        .def( py::init<>() )
        .def( "load", &PnsBridge::load, py::arg( "pcb_path" ) )
        .def( "cleanup", &PnsBridge::cleanup )                // T10
        .def( "set_mode", &PnsBridge::setMode, py::arg( "mode" ) )   // T12
        .def( "pns_layer", &PnsBridge::pnsLayer, py::arg( "board_layer" ) )
        .def( "get_obstacles", &PnsBridge::getObstacles, py::arg( "pns_layer" ) )
        .def( "get_all_obstacles", &PnsBridge::getAllObstacles )
        .def( "route_and_check", &PnsBridge::routeAndCheck, py::arg( "waypoints" ) )
        .def( "route_and_extract", &PnsBridge::routeAndExtract, py::arg( "waypoints" ) )
        .def( "nearest_unconnected", &PnsBridge::nearestUnconnected,   // T9
              py::arg( "x" ), py::arg( "y" ), py::arg( "pns_layer" ) )
        .def( "route_and_commit", &PnsBridge::routeAndCommit, py::arg( "waypoints" ) )
        .def( "probe_target", &PnsBridge::probeTarget,
              py::arg( "x" ), py::arg( "y" ), py::arg( "pns_layer" ),
              py::arg( "net" ), py::arg( "clearance" ), py::arg( "other_pns_layer" ) );
}
