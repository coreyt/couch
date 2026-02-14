cmake_minimum_required(VERSION 3.16)
project(SofaAnkleBridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- SOFA root ---
if(NOT SOFA_ROOT)
    set(SOFA_ROOT "$ENV{SOFA_ROOT}")
endif()
if(NOT SOFA_ROOT)
    message(FATAL_ERROR "SOFA_ROOT must be set (cmake -DSOFA_ROOT=... or env var)")
endif()
message(STATUS "SOFA_ROOT = ${SOFA_ROOT}")

# SOFA v24.06.00 is built with MinSizeRel; map Release -> MinSizeRel so
# imported targets resolve correctly.
set(CMAKE_MAP_IMPORTED_CONFIG_RELEASE MinSizeRel Release "")

# Tell find_package where to look
list(PREPEND CMAKE_PREFIX_PATH "${SOFA_ROOT}/lib/cmake")

# --- Dependencies ---
find_package(Eigen3 REQUIRED)
find_package(Sofa.SimpleApi REQUIRED)
# Individual component packages (Sofa.Component meta-package requires ZLIB for IO.Mesh)
find_package(Sofa.Component.StateContainer REQUIRED)
find_package(Sofa.Component.MechanicalLoad REQUIRED)
find_package(Sofa.Component.AnimationLoop REQUIRED)
find_package(Sofa.Component.Constraint.Lagrangian.Solver REQUIRED)
find_package(Sofa.Component.Constraint.Lagrangian.Correction REQUIRED)
find_package(Sofa.Component.Constraint.Projective REQUIRED)
find_package(Sofa.Component.Collision.Detection.Algorithm REQUIRED)
find_package(Sofa.Component.Collision.Detection.Intersection REQUIRED)
find_package(Sofa.Component.Collision.Geometry REQUIRED)
find_package(Sofa.Component.Collision.Response.Contact REQUIRED)
find_package(Sofa.Component.Mass REQUIRED)
find_package(Sofa.Component.ODESolver.Backward REQUIRED)
find_package(Sofa.Component.LinearSolver.Iterative REQUIRED)
find_package(Sofa.Component.Mapping.Linear REQUIRED)
find_package(Sofa.Component.Topology.Container.Constant REQUIRED)

# --- Shared library ---
add_library(SofaAnkleBridge SHARED
    src/sofa_ankle_bridge.cpp
    src/ankle_scene.cpp
)

target_include_directories(SofaAnkleBridge
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_definitions(SofaAnkleBridge PRIVATE SOFA_ANKLE_BRIDGE_EXPORTS)
set_target_properties(SofaAnkleBridge PROPERTIES
    OUTPUT_NAME SofaAnkleBridge
    # Hide symbols by default; only SOFA_BRIDGE_API symbols are exported
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    # RPATH for build tree and installed/deployed location
    BUILD_RPATH "${SOFA_ROOT}/lib"
    INSTALL_RPATH "$ORIGIN"
)

target_link_libraries(SofaAnkleBridge
    PRIVATE
        Sofa.SimpleApi
        Sofa.Component.StateContainer
        Sofa.Component.MechanicalLoad
        Sofa.Component.AnimationLoop
        Sofa.Component.Constraint.Lagrangian.Solver
        Sofa.Component.Constraint.Lagrangian.Correction
        Sofa.Component.Constraint.Projective
        Sofa.Component.Collision.Detection.Algorithm
        Sofa.Component.Collision.Detection.Intersection
        Sofa.Component.Collision.Geometry
        Sofa.Component.Collision.Response.Contact
        Sofa.Component.Mass
        Sofa.Component.ODESolver.Backward
        Sofa.Component.LinearSolver.Iterative
        Sofa.Component.Mapping.Linear
        Sofa.Component.Topology.Container.Constant
        Eigen3::Eigen
)

# --- Tests ---
option(BUILD_TESTS "Build GTest tests" ON)
if(BUILD_TESTS)
    find_package(Threads REQUIRED)
    # Use GTest bundled with SOFA
    list(PREPEND CMAKE_PREFIX_PATH "${SOFA_ROOT}/lib/cmake/GTest")
    find_package(GTest REQUIRED)

    add_executable(test_bridge test/test_bridge.cpp)
    target_link_libraries(test_bridge
        PRIVATE
            SofaAnkleBridge
            gtest  # SOFA-bundled GTest uses plain target name (no GTest:: namespace)
    )
    set_target_properties(test_bridge PROPERTIES
        BUILD_RPATH "${SOFA_ROOT}/lib;$<TARGET_FILE_DIR:SofaAnkleBridge>"
    )

    add_executable(test_ankle_scene test/test_ankle_scene.cpp)
    target_link_libraries(test_ankle_scene
        PRIVATE
            SofaAnkleBridge
            gtest
    )
    set_target_properties(test_ankle_scene PROPERTIES
        BUILD_RPATH "${SOFA_ROOT}/lib;$<TARGET_FILE_DIR:SofaAnkleBridge>"
    )

    enable_testing()
    add_test(NAME BridgeTests COMMAND test_bridge)
    set_tests_properties(BridgeTests PROPERTIES
        ENVIRONMENT "SOFA_PLUGIN_DIR=${SOFA_ROOT}/lib;SOFA_ROOT=${SOFA_ROOT}"
    )
    add_test(NAME AnkleSceneTests COMMAND test_ankle_scene)
    set_tests_properties(AnkleSceneTests PROPERTIES
        ENVIRONMENT "SOFA_PLUGIN_DIR=${SOFA_ROOT}/lib;SOFA_ROOT=${SOFA_ROOT}"
    )
endif()
