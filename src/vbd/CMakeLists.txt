add_library(novaphy_vbd STATIC
    vbd_world.cpp
    vbd_solver.cpp
)

target_include_directories(novaphy_vbd PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(novaphy_vbd PUBLIC
    novaphy_core
)

