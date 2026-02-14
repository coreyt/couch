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
find_package(Sofa.Component.StateContainer REQUIRED)

# --- Shared library ---
add_library(SofaAnkleBridge SHARED
    src/sofa_ankle_bridge.cpp
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

    enable_testing()
    add_test(NAME BridgeTests COMMAND test_bridge)
    set_tests_properties(BridgeTests PROPERTIES
        ENVIRONMENT "SOFA_PLUGIN_DIR=${SOFA_ROOT}/lib;SOFA_ROOT=${SOFA_ROOT}"
    )
endif()
