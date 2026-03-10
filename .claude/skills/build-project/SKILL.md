---
name: build-project
description: Build NovaPhy from source with correct CMake, vcpkg, and conda configuration
argument-hint: "[ipc] [clean]"
allowed-tools: Bash
---

Build NovaPhy using the standard development configuration.

## Standard Build (no IPC)

```bash
conda activate novaphy && CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=F:/vcpkg/scripts/buildsystems/vcpkg.cmake" pip install -e .
```

## IPC/CUDA Build (standalone CMake — fast incremental)

If `$ARGUMENTS` contains "ipc" or "cuda", use the standalone CMake build.
This is much faster than pip for incremental builds because it reuses
the cached CUDA objects (228 .cu files take ~1hr from scratch).

### Step 1 — Configure

Always run configure to ensure the cache is up-to-date. Pass both
`Python_ROOT_DIR` and `Python_EXECUTABLE` explicitly, and use `-U` flags
to clear stale cached Python paths from previous configures (e.g. if the
build dir was previously configured against a different Python).

```bash
conda activate novaphy
cmake -S . -B build/local-ipc-cxx20 \
  -DNOVAPHY_WITH_IPC=ON \
  -DCMAKE_TOOLCHAIN_FILE=F:/vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DCMAKE_CUDA_COMPILER=D:/CUDA/bin/nvcc.exe \
  -DPython_ROOT_DIR="C:/Users/Peng/.conda/envs/novaphy" \
  -DPython_EXECUTABLE="C:/Users/Peng/.conda/envs/novaphy/python.exe" \
  -Dpybind11_DIR="C:/Users/Peng/.conda/envs/novaphy/Lib/site-packages/pybind11/share/cmake/pybind11" \
  -DSKBUILD_PROJECT_NAME=novaphy \
  -DSKBUILD_PROJECT_VERSION=0.1.0 \
  -U_Python_EXECUTABLE \
  -U_Python_LIBRARY_RELEASE \
  -U_Python_RUNTIME_LIBRARY_RELEASE
```

**Important**: After configure, verify the cache picked up the correct
Python. The `_Python_EXECUTABLE:INTERNAL` entry matters most:

```bash
grep "_Python_EXECUTABLE:INTERNAL" build/local-ipc-cxx20/CMakeCache.txt
```

It must show `C:/Users/Peng/.conda/envs/novaphy/python.exe`. If it
shows `D:/anaconda/python.exe` (or another Python), delete the cache
and re-run configure.

### Step 2 — Build

For incremental builds (C++/binding changes only, no CUDA changes),
target just `_core` to skip CUDA recompilation entirely:

```bash
cmake --build build/local-ipc-cxx20 --config Release --target _core
```

For a full build including the CUDA backend DLL (needed on first build
or when libuipc source changes):

```bash
cmake --build build/local-ipc-cxx20 --config Release
```

**Note**: The `_core` pybind11 module does NOT link against the CUDA
backend. The backend (`uipc_backend_cuda.dll`) is loaded at runtime via
dylib. So `--target _core` skips all 228 .cu files even if they are
stale.

### Step 3 — Install the .pyd into the conda env

```bash
cmake --install build/local-ipc-cxx20 --config Release \
  --prefix "C:/Users/Peng/.conda/envs/novaphy/Lib/site-packages"
```

### Editable install finder

The editable install finder at
`C:/Users/Peng/.conda/envs/novaphy/Lib/site-packages/_novaphy_editable.py`
must point to `build/local-ipc-cxx20` (not `build/cp311-cp311-win_amd64`).
Check the last line of that file — it should read:

```python
install(..., 'E:\\NovaPhy\\build\\local-ipc-cxx20', ...)
```

If it points elsewhere, edit the path on that line.

### DLL search paths

`__init__.py` adds DLL directories automatically for both scikit-build
(`cp*-win_amd64`) and standalone build dirs (any subdir of `build/` with
`Release/bin/`). The libuipc DLLs (`uipc_core.dll`, `uipc_backend_cuda.dll`,
etc.) live in `build/local-ipc-cxx20/Release/bin/`.

### CUDA compilation progress

To monitor CUDA compilation progress during a full build:

```bash
total_cu=228
compiled=$(find "E:/NovaPhy/build/local-ipc-cxx20" -name "*.obj" -path "*cuda*" | wc -l)
echo "CUDA: $compiled / $total_cu"
```

## Clean Rebuild

If `$ARGUMENTS` contains "clean", remove the build directory first:

```bash
rm -rf build/local-ipc-cxx20
```

Then run the configure + build steps above. This forces a full CUDA
recompilation (~1hr on RTX 3060).

## After Build

1. Verify the build succeeded (check exit code)
2. Run a quick import test: `python -c "import novaphy; print('version:', novaphy.version())"`
3. If IPC build, also test: `python -c "import novaphy; print('IPC:', novaphy.has_ipc())"`
4. Report build time and any warnings
