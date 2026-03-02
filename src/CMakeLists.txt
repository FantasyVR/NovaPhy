# NovaPhy core static library

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
)

target_include_directories(novaphy_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(novaphy_core PUBLIC
    Eigen3::Eigen
)

target_compile_definitions(novaphy_core PUBLIC
    EIGEN_DEFAULT_DENSE_INDEX_TYPE=int
)
