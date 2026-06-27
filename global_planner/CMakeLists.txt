cmake_minimum_required( VERSION 3.16 )
project( gplan CXX )

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

# --- Standalone core library (no external dependencies) --------------------
add_library( planner_core STATIC planner_core.cpp )
target_include_directories( planner_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )

# --- Tiny C++ self-test (no Python needed): `cmake --build . --target selftest` ---
add_executable( selftest selftest.cpp )
target_link_libraries( selftest PRIVATE planner_core )

# --- Python module (optional; needs pybind11) ------------------------------
#   pip install pybind11
#   cmake -DBUILD_PYTHON=ON ..
option( BUILD_PYTHON "Build the Python (pybind11) module" OFF )
if( BUILD_PYTHON )
    find_package( pybind11 REQUIRED )
    pybind11_add_module( gplan bindings.cpp planner_core.cpp )
    target_include_directories( gplan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )
endif()
