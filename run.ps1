param(
    [ValidateSet("camera", "demo", "calibrate", "check", "test", "simulate")]
    [string]$Mode = "camera",
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ExtraArgs
)
$ErrorActionPreference = "Stop"
$ProjectPython = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path -LiteralPath $ProjectPython)) {
    throw "Missing .venv. Run install_environment.ps1 first."
}
Push-Location -LiteralPath $PSScriptRoot
try {
    switch ($Mode) {
        "camera" { & $ProjectPython app.py @ExtraArgs }
        "demo" { & $ProjectPython app.py --demo @ExtraArgs }
        "simulate" { & $ProjectPython step_simulator.py @ExtraArgs }
        "calibrate" { & $ProjectPython app.py --calibrate-plane @ExtraArgs }
        "check" { & $ProjectPython check_environment.py @ExtraArgs }
        "test" { & $ProjectPython -m unittest discover -s tests -v @ExtraArgs }
    }
    $RunExitCode = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $RunExitCode
