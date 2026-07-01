// bindings.cpp — pybind11 wrapper exposing the standalone planner core to Python.
//
//   import gplan
//   planner = gplan.Planner(obstacles, params)
//   paths   = planner.plan(start, start_layer, target, target_layer)

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "planner_core.h"

namespace py = pybind11;
using namespace gplan;

PYBIND11_MODULE( gplan, m )
{
    m.doc() = "Standalone global-routing path planner (pure geometry, multi-layer).";

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
        .def( py::init( []( Polygon poly, bool fixed, int layer ) {
            return Obstacle{ std::move( poly ), fixed, layer };
        } ), py::arg( "poly" ), py::arg( "fixed" ) = true, py::arg( "layer" ) = 0 )
        .def_readwrite( "poly", &Obstacle::poly )
        .def_readwrite( "fixed", &Obstacle::fixed )
        .def_readwrite( "layer", &Obstacle::layer );

    py::class_<PlannerParams>( m, "PlannerParams" )
        .def( py::init<>() )
        .def_readwrite( "trackWidth", &PlannerParams::trackWidth )
        .def_readwrite( "clearance", &PlannerParams::clearance )
        .def_readwrite( "kPaths", &PlannerParams::kPaths )
        .def_readwrite( "wCongestion", &PlannerParams::wCongestion )
        .def_readwrite( "wTightness", &PlannerParams::wTightness )
        .def_readwrite( "reusePenalty", &PlannerParams::reusePenalty )
        .def_readwrite( "layers", &PlannerParams::layers )
        .def_readwrite( "viaCost", &PlannerParams::viaCost )
        .def_readwrite( "viaClearance", &PlannerParams::viaClearance )
        .def_readwrite( "viaDiameter", &PlannerParams::viaDiameter );

    py::class_<Waypoint>( m, "Waypoint" )
        .def( py::init<>() )
        .def( py::init( []( Point p, int layer ) { return Waypoint{ p, layer }; } ),
              py::arg( "p" ), py::arg( "layer" ) = 0 )
        .def_readwrite( "p", &Waypoint::p )
        .def_readwrite( "layer", &Waypoint::layer )
        .def( "__repr__", []( const Waypoint& w ) {
            return "Waypoint((" + std::to_string( w.p.x ) + ", "
                   + std::to_string( w.p.y ) + "), L" + std::to_string( w.layer ) + ")";
        } );

    py::class_<Path>( m, "Path" )
        .def_readonly( "waypoints", &Path::waypoints )
        .def_readonly( "cost", &Path::cost );

    py::class_<Planner>( m, "Planner" )
        .def( py::init<std::vector<Obstacle>, PlannerParams>(),
              py::arg( "obstacles" ), py::arg( "params" ) )
        .def( "plan",
              py::overload_cast<Point, int, Point, int>( &Planner::plan ),
              py::arg( "start" ), py::arg( "start_layer" ),
              py::arg( "target" ), py::arg( "target_layer" ) )
        .def( "plan",
              py::overload_cast<Point, Point>( &Planner::plan ),
              py::arg( "start" ), py::arg( "target" ) )
        .def( "bump_congestion", &Planner::bumpCongestion,
              py::arg( "where" ), py::arg( "radius" ), py::arg( "factor" ) )
        .def( "clear_congestion", &Planner::clearCongestion );
}
