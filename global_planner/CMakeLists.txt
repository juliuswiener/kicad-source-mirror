cmake_minimum_required( VERSION 3.16 )
project( gplan CXX )

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

option( GPLAN_SANITIZE "Build with ASan/UBSan" OFF )
# ASan and TSan are mutually exclusive — configure them in separate build dirs.
if( GPLAN_SANITIZE AND GPLAN_TSAN )
    message( FATAL_ERROR "GPLAN_SANITIZE (ASan) and GPLAN_TSAN are incompatible; use separate build dirs." )
endif()
if( GPLAN_SANITIZE )
    add_compile_options( -fsanitize=address,undefined -g -O1 )
    add_link_options( -fsanitize=address,undefined )
endif()

# --- Standalone core library (no external dependencies) --------------------
add_library( planner_core STATIC planner_core.cpp )
target_include_directories( planner_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} )
target_compile_options( planner_core PRIVATE -Wall -Wextra )

# --- Tests + demo (no Python needed) ---------------------------------------
enable_testing()
find_package( Threads REQUIRED )

add_executable( tests tests.cpp )
target_link_libraries( tests PRIVATE planner_core Threads::Threads )
add_test( NAME core_tests COMMAND tests )

add_executable( selftest selftest.cpp )
target_link_libraries( selftest PRIVATE planner_core )
add_test( NAME selftest COMMAND selftest )

add_executable( perf_tests perf_tests.cpp )
target_link_libraries( perf_tests PRIVATE planner_core )
add_test( NAME perf_tests COMMAND perf_tests )

# Dedicated TSan build of the core tests (race check for the thread-safe Planner).
option( GPLAN_TSAN "Add a ThreadSanitizer test target" OFF )
if( GPLAN_TSAN )
    add_executable( tests_tsan tests.cpp planner_core.cpp )
    target_include_directories( tests_tsan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )
    target_compile_options( tests_tsan PRIVATE -fsanitize=thread -g -O1 )
    target_link_options( tests_tsan PRIVATE -fsanitize=thread )
    target_link_libraries( tests_tsan PRIVATE Threads::Threads )
    add_test( NAME core_tests_tsan COMMAND tests_tsan )
endif()

# --- KiCad-linked PNS bridge (pns_bridge.cpp) ------------------------------
# NOT buildable here: it links pcbnew/kicommon. Add it to your in-tree build,
# e.g. drop these sources next to qa/tools/pns and link the same libraries the
# pns regression tool links (pcbnew_kiface_objects / kicommon / pnskernel).
#   sources: pns_bridge.cpp planner_core.cpp  (+ bridge_bindings.cpp for Python)

# --- Python module (optional; needs pybind11) ------------------------------
#   pip install pybind11
#   cmake -B build -DBUILD_PYTHON=ON . && cmake --build build
option( BUILD_PYTHON "Build the Python (pybind11) module" OFF )
if( BUILD_PYTHON )
    find_package( pybind11 REQUIRED )
    pybind11_add_module( gplan bindings.cpp planner_core.cpp )
    target_include_directories( gplan PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} )
endif()
