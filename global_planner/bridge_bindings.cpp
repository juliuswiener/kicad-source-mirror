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

#include "pns_bridge.h"

namespace py = pybind11;
using namespace gbridge;

PYBIND11_MODULE( gplan_kicad, m )
{
    m.doc() = "KiCad-linked PNS bridge for the gplan global planner.";

    py::class_<RouteResult>( m, "RouteResult" )
        .def_readonly( "ok", &RouteResult::ok )
        .def_readonly( "collided", &RouteResult::collided )
        .def_readonly( "placed", &RouteResult::placed )
        .def_readonly( "vias", &RouteResult::vias )
        .def_readonly( "blocking", &RouteResult::blocking )
        .def_readonly( "reason", &RouteResult::reason );

    py::class_<PnsBridge>( m, "PnsBridge" )
        .def( py::init<>() )
        .def( "load", &PnsBridge::load, py::arg( "pcb_path" ) )
        .def( "pns_layer", &PnsBridge::pnsLayer, py::arg( "board_layer" ) )
        .def( "get_obstacles", &PnsBridge::getObstacles, py::arg( "pns_layer" ) )
        .def( "get_all_obstacles", &PnsBridge::getAllObstacles )
        .def( "route_and_check", &PnsBridge::routeAndCheck, py::arg( "waypoints" ) );
}
