# Build NovaPhy

## Build on Windows with MSVC

TODO

## Build on Linux with GCC/Clang

### Basic

NovaPhy without IPC support just needs the following prerequisites (as mentioned in [README.md](../README.md)):

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
pip install -e .

# Then you can access NovaPhy from Python in the virtual environment.
python
```

### IPC support

NovaPhy relies on [libuipc](https://github.com/spiriMirror/libuipc) for IPC support.
It requires C++20 features, and its upstream depends on some non-standard features.
Thus, the prerequisites are more specific.

- [Conda](https://docs.conda.io/) (Miniconda or Anaconda)
- [vcpkg](https://vcpkg.io/) installed.
- C++20 compiler
  - MSVC 2019+
  - GCC 11+
  - Clang 10+
- CUDA 12.4

```bash
conda activate novaphy

# Before this step, make sure your environment variables are set, such as VCPKG_ROOT.
CMAKE_ARGS="--preset=ipc" pip install -e .
```

### Troubleshooting

If you use a compiler not listed above, it may fail with the default configuration.
Here are some common reasons for crashes with specific compilers:

| Compiler | Reason | Status |
|:---:|:---|:---:|
| `gcc-9` | **libuipc** needs `<span>` which was implemented in GCC 10. | Unfixable |
| `gcc-10` | The pstl of `libstdc++ 10` isn't compatible with `onetbb` in the current baseline. (See also [vcpkg.json](../vcpkg.json)). | Unfixable |
| almost all GCC tested | `urdfdom` expects a non-standard import for `uint32_t` which is removed from `libstdc++` (may be earlier). | TODO |
| `gcc14, gcc-15` | `gcc>13` is not compatible with `nvcc-12` | Unfixable |
| `nvcc-13` | **muda** expects an unstrict dependent name lookup. | TODO |

> [!note] Clang with libstdc++
> Clang uses `libstdc++` as the default standard library. Any crash caused by `libstdc++` also affects Clang.

Compilers are described by name and major version, such as `gcc-9`. For each major version, only one version is tested:

- GCC 9.5.0
- GCC 10.5.0
- ✅ GCC 11.5.0
- ✅ GCC 12.5.0
- ✅ GCC 13.4.0
- GCC 14.3.0
- GCC 15.2.1
- ✅ nvcc-12: Cuda compilation tools, release 12.4, V12.4.131
- nvcc-13: Cuda compilation tools, release 13.1, V13.1.115
