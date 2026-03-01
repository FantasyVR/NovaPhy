---
name: run-tests
description: Run NovaPhy's Python test suite and report results
argument-hint: "[test-file-or-pattern]"
allowed-tools: Bash, Read
---

Run the NovaPhy test suite and report results clearly.

## Execution

If `$ARGUMENTS` is provided, use it as a test filter:
```bash
pytest tests/python/ -v -k "$ARGUMENTS"
```

If no arguments, run the full suite:
```bash
pytest tests/python/ -v
```

## Expected Test Count
- 46 core tests + 9 IPC tests = 55 total (IPC tests skip if not built with CUDA)

## On Failure

If any tests fail:
1. Show the full failure output with assertion details
2. Read the failing test file to understand what's expected
3. Identify the likely cause (regression, float precision, missing feature)
4. Suggest which source files to investigate

## Report Format

Summarize results as:
- Total: X passed, Y failed, Z skipped
- Failed tests (if any): list with one-line cause
- Runtime
