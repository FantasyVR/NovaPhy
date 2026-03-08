# C++ tests using Google Test

find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(novaphy_cpp_tests
    cpp/test_io_path_apis.cpp
    cpp/test_core_span_apis.cpp
)

target_link_libraries(novaphy_cpp_tests PRIVATE
    novaphy_core
    GTest::gtest_main
)

gtest_discover_tests(novaphy_cpp_tests)
