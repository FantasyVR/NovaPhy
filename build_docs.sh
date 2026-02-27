#!/usr/bin/env bash
set -euo pipefail

# pybind11 extension module to stub (NovaPhy builds `novaphy._core`)
PY_MODULE_NAME="novaphy._core"
# Replace if you want a different output location for .pyi stubs
STUB_OUTPUT_DIR="docs/stubs"

echo "[1/5] Installing documentation dependencies..."
pip install -r docs/requirements.txt

echo "[2/5] Ensuring pybind11 module is compiled/importable..."
echo "      If you already build via CMake/Setup.py manually, keep that workflow."
echo "      This script attempts an editable install for convenience."
pip install -e .

echo "[3/5] Generating Doxygen XML..."
doxygen Doxyfile

echo "[4/5] Generating Python stubs with pybind11-stubgen..."
mkdir -p "${STUB_OUTPUT_DIR}"
pybind11-stubgen "${PY_MODULE_NAME}" -o "${STUB_OUTPUT_DIR}"

echo "[5/5] Building Sphinx HTML..."
sphinx-build -b html docs docs/_build/html

echo "Done. Open docs/_build/html/index.html"
