// bindings.cpp — pybind11 wrapper exposing the standalone planner core to Python.
//
//   import gplan
//   planner = gplan.Planner(obstacles, params)
//   paths   = planner.plan(start, target)
//
// Build produces a module `gplan` (see CMakeLists.txt).

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "planner_core.h"

namespace py = pybind11;
using namespace gplan;

PYBIND11_MODULE( gplan, m )
{
    m.doc() = "Standalone global-routing path planner (pure geometry).";

    py::class_<Point>( m, "Point" )
        .def( py::init<>() )
        .def( py::init( []( double x, double y ) { return Point{ x, y }; } ) )
        .def_readwrite( "x", &Point::x )
        .def_readwrite( "y", &Point::y )
        .def( "__repr__", []( const Point& p ) {
            return "Point(" + std::to_string( p.x ) + ", " + std::to_string( p.y ) + ")";
        } );

    py::class_<Obstacle>( m, "Obstacle" )
        .def( py::init<>() )
        .def( py::init( []( Polygon poly, bool fixed ) {
            return Obstacle{ std::move( poly ), fixed };
        } ), py::arg( "poly" ), py::arg( "fixed" ) = true )
        .def_readwrite( "poly", &Obstacle::poly )
        .def_readwrite( "fixed", &Obstacle::fixed );

    py::class_<PlannerParams>( m, "PlannerParams" )
        .def( py::init<>() )
        .def_readwrite( "trackWidth", &PlannerParams::trackWidth )
        .def_readwrite( "clearance", &PlannerParams::clearance )
        .def_readwrite( "kPaths", &PlannerParams::kPaths )
        .def_readwrite( "wCongestion", &PlannerParams::wCongestion )
        .def_readwrite( "wTightness", &PlannerParams::wTightness )
        .def_readwrite( "reusePenalty", &PlannerParams::reusePenalty )
        .def_readwrite( "cornerOffset", &PlannerParams::cornerOffset );

    py::class_<Path>( m, "Path" )
        .def_readonly( "waypoints", &Path::waypoints )
        .def_readonly( "cost", &Path::cost );

    py::class_<Planner>( m, "Planner" )
        .def( py::init<std::vector<Obstacle>, PlannerParams>(),
              py::arg( "obstacles" ), py::arg( "params" ) )
        .def( "plan", &Planner::plan, py::arg( "start" ), py::arg( "target" ) )
        .def( "bump_congestion", &Planner::bumpCongestion,
              py::arg( "where" ), py::arg( "radius" ), py::arg( "factor" ) )
        .def( "clear_congestion", &Planner::clearCongestion );
}
