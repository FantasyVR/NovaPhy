---
name: run-memory-check
description: Run memory safety checks on NovaPhy using AddressSanitizer or similar tools
argument-hint: "[test-name]"
allowed-tools: Bash, Read
---

Run memory safety analysis on NovaPhy to detect leaks, buffer overflows, and use-after-free bugs.

## Windows (Primary — MSVC)

MSVC AddressSanitizer is available. Rebuild with ASAN enabled:

```bash
conda activate novaphy && CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=F:/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_CXX_FLAGS=/fsanitize=address" pip install -e . --no-build-isolation --force-reinstall
```

Then run the tests:
```bash
pytest tests/python/ -v $ARGUMENTS
```

## Linux/WSL Alternative (Valgrind)

If running under WSL or Linux:
```bash
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes python -m pytest tests/python/ -v $ARGUMENTS
```

## What to Look For

1. **Memory leaks** — Unreleased allocations, especially in Model/World lifecycle
2. **Buffer overflows** — Out-of-bounds access in collision arrays, contact buffers
3. **Use-after-free** — Dangling pointers after Model rebuild or World reset
4. **Stack overflow** — Deep recursion in articulation trees
5. **Uninitialized reads** — Uninitialized Eigen vectors/matrices

## After Analysis

1. Report all detected issues with source file and line number
2. Categorize severity (critical / warning / info)
3. Suggest fixes for each issue
4. After fixing, rebuild WITHOUT ASAN to confirm tests still pass:
   ```bash
   CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=F:/vcpkg/scripts/buildsystems/vcpkg.cmake" pip install -e . --no-build-isolation --force-reinstall
   ```
