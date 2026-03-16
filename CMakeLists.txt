cmake_minimum_required(VERSION 3.15...4.0)

# ---------- Optional IPC support via libuipc ----------
# Must be declared before project() so vcpkg installs the 'ipc' feature deps.
option(NOVAPHY_WITH_IPC "Build with libuipc IPC support (requires CUDA >= 12.4)" OFF)

if(NOVAPHY_WITH_IPC)
    list(APPEND VCPKG_MANIFEST_FEATURES "ipc")
endif()

project(
    ${SKBUILD_PROJECT_NAME}
    VERSION ${SKBUILD_PROJECT_VERSION}
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Enable CUDA when a compiler is available (for VBD and/or IPC). VBD can use
# CUDA without IPC; IPC requires CUDA.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
endif()

# Find Eigen3 via vcpkg
find_package(Eigen3 CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)

# Find pybind11 from pip-installed package (not vcpkg)
find_package(pybind11 CONFIG REQUIRED)

# Core C++ library
add_subdirectory(src)

# VBD/AVBD module (always built; uses CUDA when CMAKE_CUDA_COMPILER is set)
add_subdirectory(src/vbd)

if(NOVAPHY_WITH_IPC)
    # IPC requires CUDA and version >= 12.4 (enable_language already done above if compiler found)
    if(NOT CMAKE_CUDA_COMPILER)
        message(FATAL_ERROR "NOVAPHY_WITH_IPC requires CUDA, but no CUDA compiler was found")
    endif()
    if(CMAKE_CUDA_COMPILER_VERSION VERSION_LESS "12.4")
        message(FATAL_ERROR "NOVAPHY_WITH_IPC requires CUDA >= 12.4, found ${CMAKE_CUDA_COMPILER_VERSION}")
    endif()

    # Configure libuipc: disable components we don't need
    set(UIPC_BUILD_GUI OFF CACHE BOOL "" FORCE)
    set(UIPC_BUILD_PYBIND OFF CACHE BOOL "" FORCE)
    set(UIPC_BUILD_PYTHON_WHEEL OFF CACHE BOOL "" FORCE)
    set(UIPC_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(UIPC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(UIPC_BUILD_BENCHMARKS OFF CACHE BOOL "" FORCE)
    set(UIPC_USING_LOCAL_VCPKG ON CACHE BOOL "" FORCE)

    # Temporarily hide SKBUILD so libuipc doesn't force UIPC_BUILD_PYBIND=ON
    if(DEFINED SKBUILD)
        set(_NOVAPHY_SAVED_SKBUILD "${SKBUILD}")
        unset(SKBUILD)
        unset(SKBUILD CACHE)
    endif()

    add_subdirectory(external/libuipc EXCLUDE_FROM_ALL)

    # Workaround: VS MSBuild generator doesn't propagate Eigen include dirs
    # from transitive CMake targets to CudaCompile items in .cu files.
    # Explicitly add Eigen's include directory to the cuda backend target.
    if(TARGET cuda)
        get_target_property(_eigen_inc Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
        if(_eigen_inc)
            target_include_directories(cuda PRIVATE ${_eigen_inc})
        endif()
    endif()

    # Restore SKBUILD for NovaPhy's own use
    if(DEFINED _NOVAPHY_SAVED_SKBUILD)
        set(SKBUILD "${_NOVAPHY_SAVED_SKBUILD}" CACHE STRING "" FORCE)
    endif()
    add_subdirectory(src/ipc)

    message(STATUS "NovaPhy: IPC support ENABLED (CUDA ${CMAKE_CUDA_COMPILER_VERSION})")
else()
    message(STATUS "NovaPhy: IPC support DISABLED (set -DNOVAPHY_WITH_IPC=ON to enable)")
endif()

# Python bindings
add_subdirectory(python/bindings)

# Tests (only when building standalone, not via pip install)
if(NOT DEFINED SKBUILD)
    find_package(GTest CONFIG QUIET)
    if(GTest_FOUND)
        enable_testing()
        add_subdirectory(tests)
    endif()
endif()
