# NovaPhy IPC adapter library (C++20, links libuipc)

add_library(novaphy_ipc STATIC
    ipc_world.cpp
    shape_converter.cpp
)

# This target must compile as C++20 to match libuipc
set_target_properties(novaphy_ipc PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
)

target_include_directories(novaphy_ipc PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(novaphy_ipc 
    PUBLIC
        novaphy_core
    PRIVATE
        uipc::uipc
)

# Ensure libuipc backend DLLs are built (they are MODULE libraries
# not linked into uipc::uipc, so EXCLUDE_FROM_ALL would skip them)
if(TARGET uipc::backends)
    add_dependencies(novaphy_ipc uipc::backends)
endif()
