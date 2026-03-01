---
name: run-linter
description: Run code formatting and static analysis on NovaPhy source files
argument-hint: "[path-or-check]"
allowed-tools: Bash, Read, Grep, Glob
---

Run code quality checks on NovaPhy's C++ and Python source code.

## C++ Formatting (clang-format)

Check formatting on all C++ source files:
```bash
find include/novaphy src python/bindings -name "*.h" -o -name "*.cpp" | xargs clang-format --dry-run --Werror 2>&1
```

To auto-fix formatting:
```bash
find include/novaphy src python/bindings -name "*.h" -o -name "*.cpp" | xargs clang-format -i
```

## C++ Static Analysis (clang-tidy)

Run clang-tidy on source files (requires compile_commands.json):
```bash
# Generate compile_commands.json if not present
CMAKE_ARGS="-DCMAKE_TOOLCHAIN_FILE=F:/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON" pip install -e .

# Run clang-tidy
clang-tidy src/**/*.cpp -p build/ --checks="-*,bugprone-*,performance-*,modernize-*,readability-*" 2>&1
```

## Python Linting

```bash
# Ruff (fast Python linter)
ruff check python/ tests/ demos/ --select=E,W,F,I

# Type checking (if mypy is available)
mypy python/novaphy/ --ignore-missing-imports
```

## Targeted Check

If `$ARGUMENTS` is provided:
- If it's a file path, lint only that file
- If it's "format" or "fix", auto-fix formatting issues
- If it's "python", run only Python linters
- If it's "cpp", run only C++ checks

## Report

1. Group issues by severity: errors, warnings, style
2. Show file:line for each issue
3. For auto-fixable issues, ask before applying fixes
4. Verify that fixes don't break compilation: rebuild and run tests
