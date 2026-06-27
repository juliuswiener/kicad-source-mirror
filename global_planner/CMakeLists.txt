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

# --- KiCad-linked PNS bridge (pns_bridge.cpp) ------------------------------
# NOT buildable here: it links pcbnew/kicommon. Add it to your in-tree build,
# e.g. drop these sources next to qa/tools/pns and link the same libraries the
# pns regression tool links (pcbnew_kiface_objects / kicommon / pnskernel).
#   sources: pns_bridge.cpp planner_core.cpp  (+ bridge_bindings.cpp for Python)

# --- Python module (optional; needs pybind11) ------------------------------
#   pip install pybind11
#   cmake -DBUILD_PYTHON=ON ..
option( BUILD_PYTHON "Build the Python (pybind11) module" OFF )
if( BUILD_PYTHON )
    find_package( pybind11 REQUIRED )
    pybind11_add_module( gplan bindings.cpp planner_core.cpp )
    target_include_directories( gplan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )
endif()
