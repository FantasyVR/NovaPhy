# Build NovaPhy

[TOC]

## Build on Windows with MSVC

TODO

## Build on Linux with GCC/Clang

### Basic

NovaPhy without IPC support only requires the following prerequisites (as mentioned in [README.md](../README.md#setup)):

- [Conda](https://docs.conda.io/) (Miniconda or Anaconda)
- [vcpkg](https://vcpkg.io/) installed. *Optional*
- C++17 compiler
  - MSVC 2019+
  - GCC 9+
  - Clang 10+

```bash
# Create conda environment
conda env create -f environment.yml
conda activate novaphy

# Install NovaPhy
CMAKE_ARGS="--preset=default" pip install -e .

# Then you can access NovaPhy from Python in the virtual environment.
python
```

### IPC support

NovaPhy relies on [libuipc](https://github.com/spiriMirror/libuipc) for IPC support.
It requires C++20 features, and some of its upstream dependencies rely on non-standard features.
Thus, the prerequisites are more specific.

- [Conda](https://docs.conda.io/) (Miniconda or Anaconda)
- [vcpkg](https://vcpkg.io/) installed.
- C++20 compiler
  - MSVC 2019+
  - GCC 11, 12, 13
  - Clang 10+
- CUDA 12.4

```bash
conda activate novaphy

# Before this step, make sure your environment variables are set (for example, VCPKG_ROOT).
git submodule update --init --recursive # Update submodules.
CMAKE_ARGS="--preset=ipc" pip install -e .
```

### Troubleshooting

If you use a compiler not listed above, the default configuration may fail.
Here are some common causes of failures with specific compilers:

| Compiler | Reason | Status |
|:---:|:---|:---:|
| `gcc-9` | **libuipc** needs `<span>`, which was introduced in GCC 10. | Unfixable |
| `gcc-10` | The PSTL in `libstdc++ 10` isn't compatible with `onetbb` in the current baseline. (See also [vcpkg.json](../vcpkg.json).) | Unfixable |
| `gcc-15` | `urdfdom` expects a non-standard import for `uint32_t`, which has been removed from `libstdc++`. | Fixed [^1] |
| `gcc-14, gcc-15` | `gcc>13` is not compatible with `nvcc-12` | Unfixable |
| `nvcc-13` | **muda** expects non-strict dependent name resolution. | TODO |

[^1]: The issue has been fixed by upstream maintainers with a patch. However, the patch has not been included in the current baseline (2025.07.25). Adding `-include cstdint` to your `CXXFLAGS` environment variable should resolve the problem.

> [!note] Clang with libstdc++
> Clang uses `libstdc++` as the default standard library. Any crash caused by `libstdc++` also affects Clang.

Compilers are described by name and major version, such as `gcc-9`. For each major version, only one version is tested (✅ marks compatible compilers):

- GCC 9.5.0
- GCC 10.5.0
- ✅ GCC 11.5.0
- ✅ GCC 12.5.0
- ✅ GCC 13.4.0
- GCC 14.3.0
- GCC 15.2.1
- ✅ nvcc-12: CUDA compilation tools, release 12.4, V12.4.131
- nvcc-13: CUDA compilation tools, release 13.1, V13.1.115

## Components Packaging and Installation

Overall, NovaPhy has two components:

- A Python binding as the frontend
- A C++ library as the backend

Because the backend is statically linked to the Python extension in this project, any package that includes the frontend also includes the backend. However, the backend can also be built and distributed independently.

Two methods are provided for handling the backend's largest dependency, `libuipc`: bundled and standalone. The bundled method means `libuipc` will be included in the backend component, and NovaPhy will not rely on `libuipc` from the system or virtual environment. The standalone method means `libuipc` will be imported from the environment, and the binary target of `libuipc` will not be included in the package. Note that `libuipc` in this context refers to its C++ library only; the Python binding will never be included in the NovaPhy wheel package. Although we allow `libuipc` to be detached from NovaPhy, its incomplete CMake packaging support makes it difficult to use the NovaPhy C++ library without a bundled `libuipc`. Use CMake option `NOVAPHY_BUNDLE_UIPC` to control the behavior.

There are two packaging methods:

- Building with standard Python build tools, using `scikit-build-core` as the build backend.
- Building with CMake without `scikit-build-core`, which allows you to build a C/C++ package.

### Package with Python build tools

Using `build` with `scikit-build-core` will build and package the two components of NovaPhy together into a wheel package. This is convenient for Python developers.

```bash
pip install build
CMAKE_ARGS="--preset=ipc" python -m build

# Or, use libuipc from the environment
CMAKE_ARGS="--preset=ipc -DNOVAPHY_BUNDLE_UIPC=OFF" python -m build
```

### Standalone CMake

To use NovaPhy in a C++ program, you can build it as a standard CMake project. You can install NovaPhy on your machine using CMake (as shown below), or use `add_subdirectory(/path/to/NovaPhy)` directly.

```bash
mkdir build
cd build
cmake -S .. -B . --preset=default --install-prefix=/path/to/installation
# or
cmake -S .. -B . --preset=ipc ... # to enable ipc support

# build
cmake --build .

# install
cmake --install . --component core
```
