$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

function Resolve-DoxygenExe {
    $cmd = Get-Command doxygen -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }

    if ($env:DOXYGEN_EXE -and (Test-Path $env:DOXYGEN_EXE)) { return $env:DOXYGEN_EXE }

    $candidates = @(
        "D:\doxygen\bin\doxygen.exe",
        "$env:ProgramFiles\Doxygen\bin\doxygen.exe",
        "$env:ProgramFiles\doxygen\bin\doxygen.exe",
        "$env:ProgramFiles(x86)\Doxygen\bin\doxygen.exe",
        "$env:ProgramFiles(x86)\doxygen\bin\doxygen.exe",
        "C:\ProgramData\chocolatey\bin\doxygen.exe",
        "$env:USERPROFILE\scoop\apps\doxygen\current\bin\doxygen.exe"
    )

    foreach ($p in $candidates) {
        if ($p -and (Test-Path $p)) { return $p }
    }

    throw "Could not find doxygen.exe. Add it to PATH or set DOXYGEN_EXE to the full path."
}

$doxygen = Resolve-DoxygenExe
Write-Host ("Using Doxygen: {0}" -f $doxygen)

Write-Host "[1/5] Installing documentation dependencies..."
python -m pip install -r docs\requirements.txt

Write-Host "[2/5] Ensuring pybind11 module is compiled/importable..."
python -m pip install -e .

Write-Host "[3/5] Generating Doxygen XML..."
& $doxygen Doxyfile

Write-Host "[4/5] Generating Python stubs with pybind11-stubgen..."
New-Item -ItemType Directory -Force docs\stubs | Out-Null
python -m pybind11_stubgen novaphy._core -o docs\stubs

Write-Host "[5/5] Building Sphinx HTML..."
python -m sphinx -b html docs docs\_build\html

Write-Host "Done. Open docs\\_build\\html\\index.html"
