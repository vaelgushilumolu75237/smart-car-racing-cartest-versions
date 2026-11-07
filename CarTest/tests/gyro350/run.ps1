param(
    [string]$ZigPath = 'D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe'
)

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$testExe = Join-Path $env:TEMP 'gyro350-pure-c-tests.exe'

if (-not (Test-Path -LiteralPath $ZigPath -PathType Leaf)) {
    throw "Zig compiler not found: $ZigPath"
}

Push-Location $repoRoot
try {
    $variants = @(
        @{ Name = 'positive'; Define = $null },
        @{ Name = 'negative'; Define = '-DCAR_IMU_YAW_SIGN=-1' }
    )
    foreach ($variant in $variants) {
        $variantExe = Join-Path $env:TEMP ("gyro350-pure-c-tests-" + $variant.Name + ".exe")
        $compileArgs = @(
            'cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', 'Inc'
        )
        if ($null -ne $variant.Define) {
            $compileArgs += $variant.Define
        }
        $compileArgs += @(
            'App\icm42688.c', 'App\gyro_ring.c', 'tests\gyro350\test_gyro350.c',
            '-o', $variantExe
        )
        & $ZigPath @compileArgs
        if ($LASTEXITCODE -ne 0) {
            throw ("Zig host compile failed for {0} with exit code {1}" -f
                $variant.Name, $LASTEXITCODE)
        }
        & $variantExe
        if ($LASTEXITCODE -ne 0) {
            throw ("gyro350 host tests failed for {0} with exit code {1}" -f
                $variant.Name, $LASTEXITCODE)
        }
    }
}
finally {
    Pop-Location
}
