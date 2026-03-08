cmake_minimum_required(VERSION 3.15...4.0)

# ---------- Optional IPC support via libuipc ----------
# Must be declared before project() so vcpkg installs the 'ipc' feature deps.
option(NOVAPHY_WITH_IPC "Build with libuipc IPC support (requires CUDA >= 12.4)" OFF)
option(NOVAPHY_BUNDLE_UIPC "Bundle libuipc with NovaPhy package. If disabled, novaphy will attempt to find it system-wide." ON)
option(NOVAPHY_ENABLE_INSTALL "Enable installation of NovaPhy targets and CMake config files" ON)

if(NOVAPHY_WITH_IPC)
    list(APPEND VCPKG_MANIFEST_FEATURES "ipc")
endif()

if (NOVAPHY_BUNDLE_UIPC AND NOT NOVAPHY_WITH_IPC)
    message(WARNING "NOVAPHY_BUNDLE_UIPC is enabled but NOVAPHY_WITH_IPC is disabled. Ignoring NOVAPHY_BUNDLE_UIPC.")
    set(NOVAPHY_BUNDLE_UIPC OFF CACHE BOOL "" FORCE)
endif()

# ---------- Project setup ----------
set(NOVAPHY_PROJECT_NAME "")
set(NOVAPHY_PROJECT_VERSION "")
if (SKBUILD)
    set(NOVAPHY_PROJECT_NAME "${SKBUILD_PROJECT_NAME}")
    set(NOVAPHY_PROJECT_VERSION "${SKBUILD_PROJECT_VERSION}")
else()
    set(NOVAPHY_PROJECT_NAME "NovaPhy")
    # TODO There are too many version strings floating around (setup.py, CMakeLists.txt, __init__.py).
    set(NOVAPHY_PROJECT_VERSION "0.1.0")
endif()

# Disable install and test when building as a subproject.
if (NOT CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    set(NOVAPHY_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    # TODO NovaPhy doesn't have any C++ tests for now.
endif()

include(compat/compat.cmake)

project(
    ${NOVAPHY_PROJECT_NAME}
    VERSION ${NOVAPHY_PROJECT_VERSION}
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find Eigen3 via vcpkg
find_package(Eigen3 CONFIG REQUIRED)

# Find pybind11 from pip-installed package (not vcpkg)
find_package(pybind11 CONFIG REQUIRED)

# Core C++ library
add_subdirectory(src)

if(NOVAPHY_WITH_IPC)
    # CUDA is required by libuipc
    include(CheckLanguage)
    check_language(CUDA)
    if(CMAKE_CUDA_COMPILER)
        enable_language(CUDA)
        if(CMAKE_CUDA_COMPILER_VERSION VERSION_LESS "12.4")
            message(FATAL_ERROR "NOVAPHY_WITH_IPC requires CUDA >= 12.4, found ${CMAKE_CUDA_COMPILER_VERSION}")
        endif()
    else()
        message(FATAL_ERROR "NOVAPHY_WITH_IPC requires CUDA, but no CUDA compiler was found")
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

# Install CMake config files
if (NOVAPHY_ENABLE_INSTALL AND NOT SKBUILD)
    include(CMakePackageConfigHelpers)
    write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/novaphyConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
    )

    file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/novaphyConfig.cmake" 
    [====[
        include(${CMAKE_CURRENT_LIST_DIR}/novaphyCoreTargets.cmake)
    ]====])

    install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/novaphyConfigVersion.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/novaphyConfig.cmake"
        DESTINATION lib/cmake/novaphy
        COMPONENT core
    )
endif()