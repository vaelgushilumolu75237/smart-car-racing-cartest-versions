param(
    [string]$Python = "python"
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
Push-Location $repoRoot
try {
    & $Python -m unittest discover -s tests\k230_gyro -p "test_*.py" -v
    if ($LASTEXITCODE -ne 0) {
        throw "K230 gyro-link tests failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
