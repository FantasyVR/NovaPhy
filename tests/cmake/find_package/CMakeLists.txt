# The sample C++ Project use NovaPhy by find_package

cmake_minimum_required(VERSION 3.15..4)

project(NovaPhyFindPackageTest LANGUAGES CXX)

find_package(novaphy CONFIG REQUIRED)

# Export the novaphy::core target for use in this project

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE novaphy::core)