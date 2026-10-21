param([string]$Python = "python", [switch]$Simulation)
$ErrorActionPreference = "Stop"
Push-Location -LiteralPath $PSScriptRoot
try {
    & $Python -c "import sys,struct; assert struct.calcsize('P')==8, 'Use 64-bit Python'; assert (3,11)<=sys.version_info[:2]<=(3,13), 'Use Python 3.11-3.13; 3.13 is verified'"
    if ($LASTEXITCODE -ne 0) { throw "Python version/architecture check failed." }
    & $Python -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw "Virtual environment creation failed." }
    $ProjectPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
    & $ProjectPython -m pip install --only-binary=:all: --no-cache-dir -r requirements-lock.txt
    if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed." }
    if ($Simulation) {
        & $ProjectPython -m pip install --only-binary=:all: --constraint requirements-lock.txt -r requirements-simulation.txt
        if ($LASTEXITCODE -ne 0) { throw "STEP simulation dependency installation failed." }
    }
    & $ProjectPython -m pip check
    if ($LASTEXITCODE -ne 0) { throw "Dependency compatibility check failed." }
    & $ProjectPython check_environment.py --gui
    if ($LASTEXITCODE -ne 0) { throw "Runtime check failed." }
    Write-Host "Environment ready. Run .\run.ps1 demo or .\run.ps1 camera."
} finally {
    Pop-Location
}
