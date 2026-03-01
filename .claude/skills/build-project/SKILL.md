---
name: build-project
description: Build NovaPhy from source with correct CMake, vcpkg, and conda configuration
argument-hint: "[ipc]"
allowed-tools: Bash
---

Build NovaPhy using the standard development configuration.

## Standard Build

```bash
conda activate novaphy && CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=F:/vcpkg/scripts/buildsystems/vcpkg.cmake" pip install -e .
```

## IPC/CUDA Build

If `$ARGUMENTS` contains "ipc" or "cuda", use the IPC-enabled build instead:

```bash
conda activate novaphy && CMAKE_ARGS="-DNOVAPHY_WITH_IPC=ON -DCMAKE_TOOLCHAIN_FILE=F:/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_CUDA_COMPILER=D:/CUDA/bin/nvcc.exe" pip install -e .
```

## Clean Rebuild

If `$ARGUMENTS` contains "clean", add `--no-build-isolation --force-reinstall`:

```bash
pip install -e . --no-build-isolation --force-reinstall
```

## After Build

1. Verify the build succeeded (check exit code)
2. Run a quick import test: `python -c "import novaphy; print('OK:', dir(novaphy))"`
3. If IPC build, also test: `python -c "from novaphy._core import IPCWorld; print('IPC OK')"`
4. Report build time and any warnings
