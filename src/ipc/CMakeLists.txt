# NovaPhy IPC adapter library (links libuipc)

add_library(novaphy_ipc STATIC
    ipc_world.cpp
    shape_converter.cpp
)

target_include_directories(novaphy_ipc PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(novaphy_ipc PUBLIC
    novaphy_core
    uipc::uipc
)

target_compile_definitions(novaphy_ipc PUBLIC
    NOVAPHY_HAS_IPC=1
)

# Ensure libuipc backend DLLs are built (they are MODULE libraries
# not linked into uipc::uipc, so EXCLUDE_FROM_ALL would skip them)
if(TARGET uipc::backends)
    add_dependencies(novaphy_ipc uipc::backends)
endif()
