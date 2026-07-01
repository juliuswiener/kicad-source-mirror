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

    py::class_<DragProbe>( m, "DragProbe" )
        .def_readonly( "clean", &DragProbe::clean )
        .def_readonly( "cost", &DragProbe::cost )
        .def_readonly( "shoved", &DragProbe::shoved );

    py::class_<ShoveResult>( m, "ShoveResult" )
        .def_readonly( "committed", &ShoveResult::committed )
        .def_readonly( "x", &ShoveResult::x )
        .def_readonly( "y", &ShoveResult::y )
        .def_readonly( "cost", &ShoveResult::cost )
        .def_readonly( "change", &ShoveResult::change );

    py::class_<EscapeClearResult>( m, "EscapeClearResult" )
        .def_readonly( "ok", &EscapeClearResult::ok )
        .def_readonly( "iterations", &EscapeClearResult::iterations )
        .def_readonly( "moves", &EscapeClearResult::moves )
        .def_readonly( "blocking", &EscapeClearResult::blocking );

    py::class_<PnsBridge::TuneResult>( m, "TuneResult" )
        .def_readonly( "change", &PnsBridge::TuneResult::change )
        .def_readonly( "status", &PnsBridge::TuneResult::status )
        .def_readonly( "current_length", &PnsBridge::TuneResult::currentLength )
        .def_readonly( "target_length", &PnsBridge::TuneResult::targetLength );

    auto bridge = py::class_<PnsBridge>( m, "PnsBridge" );
    py::enum_<PnsBridge::RouteMode>( bridge, "RouteMode" )   // T12
        .value( "MARK_OBSTACLES", PnsBridge::RouteMode::MarkObstacles )
        .value( "SHOVE",          PnsBridge::RouteMode::Shove )
        .value( "WALKAROUND",     PnsBridge::RouteMode::Walkaround );
    bridge
        .def( py::init<>() )
        .def( "load", []( PnsBridge& self, const std::string& p ) { return self.load( p ); },
              py::arg( "pcb_path" ) )
        // 0.6 — like load(), but returns (ok, error_detail) instead of a bare
        // bool so callers can distinguish project-load / parse / attach failure.
        .def( "load_ex", []( PnsBridge& self, const std::string& p )
              {
                  std::string err;
                  bool ok = self.load( p, &err );
                  return py::make_tuple( ok, err );
              }, py::arg( "pcb_path" ) )
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
              py::arg( "net" ), py::arg( "clearance" ), py::arg( "other_pns_layer" ) )
        .def( "drag_component", &PnsBridge::dragComponent,
              py::arg( "x" ), py::arg( "y" ), py::arg( "new_x" ), py::arg( "new_y" ),
              py::arg( "allow_violations" ) = false )
        .def( "route_long_haul", &PnsBridge::routeLongHaul,
              py::arg( "waypoints" ), py::arg( "max_inserts" ) = 6 )
        .def( "probe_drag", &PnsBridge::probeDrag,
              py::arg( "x" ), py::arg( "y" ), py::arg( "new_x" ), py::arg( "new_y" ) )
        .def( "shove_component_search", &PnsBridge::shoveComponentSearch,
              py::arg( "x" ), py::arg( "y" ), py::arg( "candidates" ) )
        .def( "route_diff_pair_and_commit", &PnsBridge::routeDiffPairAndCommit,
              py::arg( "waypoints" ) )
        .def( "tune_length", &PnsBridge::tuneLength,
              py::arg( "x" ), py::arg( "y" ), py::arg( "end_x" ), py::arg( "end_y" ),
              py::arg( "pns_layer" ), py::arg( "target_length_nm" ) )
        .def( "probe_via_move", &PnsBridge::probeViaMove,
              py::arg( "x" ), py::arg( "y" ), py::arg( "new_x" ), py::arg( "new_y" ) )
        .def( "move_via", &PnsBridge::moveVia,
              py::arg( "x" ), py::arg( "y" ), py::arg( "new_x" ), py::arg( "new_y" ),
              py::arg( "allow_violations" ) = false )
        .def( "shove_via_search", &PnsBridge::shoveViaSearch,
              py::arg( "x" ), py::arg( "y" ), py::arg( "candidates" ) )
        .def( "clear_escape_corridor", &PnsBridge::clearEscapeCorridor,
              py::arg( "x" ), py::arg( "y" ), py::arg( "pns_layer" ),
              py::arg( "dir_x" ), py::arg( "dir_y" ), py::arg( "radius" ),
              py::arg( "step_nm" ) = 100000.0, py::arg( "max_iterations" ) = 8 );
}
