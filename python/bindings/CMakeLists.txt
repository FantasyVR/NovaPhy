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

# VBD/AVBD bindings (always built; implementation currently)
list(APPEND BIND_SOURCES bind_vbd.cpp)

pybind11_add_module(_core ${BIND_SOURCES})

target_link_libraries(_core PRIVATE
    novaphy_core
)

if(NOVAPHY_WITH_IPC)
    target_link_libraries(_core PRIVATE novaphy_ipc)
    target_compile_definitions(_core PRIVATE NOVAPHY_HAS_IPC=1)
endif()

# Link VBD/AVBD (Windows: STATIC like novaphy_ipc; Unix: SHARED with RPATH below)
target_link_libraries(_core PRIVATE novaphy_vbd)

# So the loader finds libnovaphy_vbd.so from build tree and when installed (CI / editable)
if(UNIX)
    set_target_properties(_core PROPERTIES
        BUILD_RPATH "$<TARGET_FILE_DIR:novaphy_vbd>"
        INSTALL_RPATH "$ORIGIN"
    )
endif()

target_compile_definitions(_core PRIVATE
    VERSION_INFO=${PROJECT_VERSION}
)

install(TARGETS _core DESTINATION novaphy)

# Install VBD shared library only on Unix (on Windows novaphy_vbd is STATIC and linked into _core).
if(TARGET novaphy_vbd AND NOT MSVC)
    install(TARGETS novaphy_vbd DESTINATION novaphy)
endif()
