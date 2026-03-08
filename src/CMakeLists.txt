# NovaPhy core static library

configure_file(
    ${CMAKE_SOURCE_DIR}/include/novaphy/novaphy_config.h.in
    ${CMAKE_BINARY_DIR}/include/novaphy/novaphy_config.h
)

add_library(novaphy_core STATIC
    math/spatial.cpp
    math/math_utils.cpp
    core/body.cpp
    core/shape.cpp
    core/model_builder.cpp
    core/joint.cpp
    collision/broadphase.cpp
    collision/narrowphase.cpp
    dynamics/integrator.cpp
    dynamics/free_body_solver.cpp
    dynamics/featherstone.cpp
    dynamics/articulated_solver.cpp
    sim/state.cpp
    sim/world.cpp
    fluid/particle_state.cpp
    fluid/neighbor_search.cpp
    fluid/pbf_solver.cpp
    fluid/fluid_world.cpp
    fluid/boundary.cpp
)

add_library(novaphy::core ALIAS novaphy_core)

target_include_directories(novaphy_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
)

target_link_libraries(novaphy_core PUBLIC
    Eigen3::Eigen
)

target_compile_definitions(novaphy_core PUBLIC
    EIGEN_DEFAULT_DENSE_INDEX_TYPE=int
)

if (NOVAPHY_ENABLE_INSTALL)
    if (NOT SKBUILD)
        # For CMake Standalone Build --- install the library and headers.
        # `novaphy_core` is a static library that would be linked into the Python extension module.
        # It's meaningless to package it into Python wheels.

        target_include_directories(novaphy_core PUBLIC
            $<INSTALL_INTERFACE:include>
        )

        set_target_properties(novaphy_core PROPERTIES
            EXPORT_NAME core
        )

        install(TARGETS novaphy_core
            COMPONENT core
            EXPORT novaphyTargets
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
        )

        install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/novaphy/
            COMPONENT core
            DESTINATION include/novaphy
            FILES_MATCHING PATTERN "*.h"
        )

        install(DIRECTORY ${CMAKE_BINARY_DIR}/include/novaphy/
            COMPONENT core
            DESTINATION include/novaphy
            FILES_MATCHING PATTERN "*.h"
        )

        install(EXPORT novaphyTargets
            COMPONENT core
            FILE novaphyCoreTargets.cmake
            NAMESPACE novaphy::
            DESTINATION lib/cmake/novaphy
        )

    endif()

    if (NOVAPHY_BUNDLE_UIPC)
        # If we're building a bundled libuipc, we need to export its targets as well.
        # TODO Some configuration is needed to avoid conflicts when libuipc is also installed separately on the system.

        if (SKBUILD)
            set(uipc_bundle_dest novaphy/lib/novaphy-bundled-libs)
        else()
            set(uipc_bundle_dest lib/novaphy-bundled-libs)
        endif()

        install(DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin/"
            COMPONENT core
            DESTINATION ${uipc_bundle_dest}
            FILES_MATCHING PATTERN "*"
        )
    endif()
endif()