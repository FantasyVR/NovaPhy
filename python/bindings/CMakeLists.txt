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
    target_compile_definitions(_core PRIVATE NOVAPHY_HAS_IPC=1)
endif()

target_compile_definitions(_core PRIVATE
    VERSION_INFO=${PROJECT_VERSION}
)

install(TARGETS _core DESTINATION novaphy)
