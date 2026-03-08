# Python bindings via pybind11

set(BIND_SOURCES
    bind_main.cpp
    bind_math.cpp
    bind_core.cpp
    bind_collision.cpp
    bind_performance.cpp
    bind_sim.cpp
    bind_dynamics.cpp
    bind_fluid.cpp
    bind_io.cpp
)

if(NOVAPHY_WITH_IPC)
    list(APPEND BIND_SOURCES bind_ipc.cpp)
endif()

pybind11_add_module(_core ${BIND_SOURCES})

target_link_libraries(_core PRIVATE
    novaphy_core
)

if(NOVAPHY_WITH_IPC)
    target_link_libraries(_core PRIVATE novaphy_ipc)
    # C++20 needed for bind_ipc.cpp which includes libuipc headers
    set_source_files_properties(bind_ipc.cpp PROPERTIES
        COMPILE_FLAGS "$<$<CXX_COMPILER_ID:MSVC>:/std:c++20>$<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-std=c++20>"
    )
    target_compile_definitions(_core PRIVATE NOVAPHY_HAS_IPC=1)
endif()

target_compile_definitions(_core PRIVATE
    VERSION_INFO=${PROJECT_VERSION}
)

install(TARGETS _core DESTINATION novaphy)
