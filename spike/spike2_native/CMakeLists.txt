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
find_package(Threads REQUIRED)

add_library(SofaAnkleBridge SHARED
    src/sofa_ankle_bridge.cpp
    src/ankle_scene.cpp
    src/scene_builder.cpp
    src/thread_manager.cpp
)

target_include_directories(SofaAnkleBridge
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
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
        Threads::Threads
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

    add_executable(test_async test/test_async.cpp)
    target_link_libraries(test_async
        PRIVATE
            SofaAnkleBridge
            gtest
    )
    set_target_properties(test_async PROPERTIES
        BUILD_RPATH "${SOFA_ROOT}/lib;$<TARGET_FILE_DIR:SofaAnkleBridge>"
    )

    add_executable(test_scene_builder test/test_scene_builder.cpp)
    target_link_libraries(test_scene_builder
        PRIVATE
            SofaAnkleBridge
            gtest
    )
    set_target_properties(test_scene_builder PROPERTIES
        BUILD_RPATH "${SOFA_ROOT}/lib;$<TARGET_FILE_DIR:SofaAnkleBridge>"
    )

    add_executable(test_triple_buffer test/test_triple_buffer.cpp)
    target_include_directories(test_triple_buffer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    target_link_libraries(test_triple_buffer
        PRIVATE
            SofaAnkleBridge
            gtest
            Threads::Threads
    )
    set_target_properties(test_triple_buffer PROPERTIES
        BUILD_RPATH "${SOFA_ROOT}/lib;$<TARGET_FILE_DIR:SofaAnkleBridge>"
    )

    add_executable(test_joint_angles test/test_joint_angles.cpp)
    target_link_libraries(test_joint_angles
        PRIVATE
            SofaAnkleBridge
            gtest
    )
    set_target_properties(test_joint_angles PROPERTIES
        BUILD_RPATH "${SOFA_ROOT}/lib;$<TARGET_FILE_DIR:SofaAnkleBridge>"
    )

    add_executable(test_rom_validation test/test_rom_validation.cpp)
    target_link_libraries(test_rom_validation
        PRIVATE
            SofaAnkleBridge
            gtest
    )
    set_target_properties(test_rom_validation PROPERTIES
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
    add_test(NAME AsyncTests COMMAND test_async)
    set_tests_properties(AsyncTests PROPERTIES
        ENVIRONMENT "SOFA_PLUGIN_DIR=${SOFA_ROOT}/lib;SOFA_ROOT=${SOFA_ROOT}"
    )
    add_test(NAME SceneBuilderTests COMMAND test_scene_builder)
    set_tests_properties(SceneBuilderTests PROPERTIES
        ENVIRONMENT "SOFA_PLUGIN_DIR=${SOFA_ROOT}/lib;SOFA_ROOT=${SOFA_ROOT}"
    )
    add_test(NAME TripleBufferTests COMMAND test_triple_buffer)
    set_tests_properties(TripleBufferTests PROPERTIES
        ENVIRONMENT "SOFA_PLUGIN_DIR=${SOFA_ROOT}/lib;SOFA_ROOT=${SOFA_ROOT}"
    )
    add_test(NAME JointAngleTests COMMAND test_joint_angles)
    set_tests_properties(JointAngleTests PROPERTIES
        ENVIRONMENT "SOFA_PLUGIN_DIR=${SOFA_ROOT}/lib;SOFA_ROOT=${SOFA_ROOT}"
    )
    add_test(NAME ROMValidationTests COMMAND test_rom_validation)
    set_tests_properties(ROMValidationTests PROPERTIES
        ENVIRONMENT "SOFA_PLUGIN_DIR=${SOFA_ROOT}/lib;SOFA_ROOT=${SOFA_ROOT}"
    )
endif()
