param(
    [string]$ZigPath = 'D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe'
)

$ErrorActionPreference = 'Stop'

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
if (-not (Test-Path -LiteralPath $ZigPath -PathType Leaf)) {
    throw "Zig compiler not found: $ZigPath"
}

Push-Location $repoRoot
try {
    $profiles = @(
        @{ Name = 'A1'; Defines = @('-DCARTEST_TRACK_PROFILE=1') },
        @{ Name = 'B2'; Defines = @('-DCARTEST_TRACK_PROFILE=2') },
        @{ Name = 'C100'; Defines = @('-DCARTEST_TRACK_PROFILE=3', '-DCARTEST_SPEED_PERCENT=100') },
        @{ Name = 'C120'; Defines = @('-DCARTEST_TRACK_PROFILE=3', '-DCARTEST_SPEED_PERCENT=120') },
        @{ Name = 'C130'; Defines = @('-DCARTEST_TRACK_PROFILE=3', '-DCARTEST_SPEED_PERCENT=130') },
        @{ Name = 'C140'; Defines = @('-DCARTEST_TRACK_PROFILE=3', '-DCARTEST_SPEED_PERCENT=140') }
    )
    foreach ($profile in $profiles) {
        foreach ($sign in @(
            @{ Name = 'positive'; Define = $null },
            @{ Name = 'negative'; Define = '-DCAR_IMU_YAW_SIGN=-1' }
        )) {
            $variantName = $profile.Name + '-' + $sign.Name
            $variantExe = Join-Path $env:TEMP ("gyro350-pure-c-tests-" + $variantName + ".exe")
            $compileArgs = @(
                'cc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-I', 'Inc', '-DCAR_GYRO_RING_APPROACH_MS=0u'
            )
            $compileArgs += $profile.Defines
            if ($null -ne $sign.Define) {
                $compileArgs += $sign.Define
            }
            $compileArgs += @(
                'App\icm42688.c', 'App\gyro_ring.c', 'tests\gyro350\test_gyro350.c',
                '-o', $variantExe
            )
            & $ZigPath @compileArgs
            if ($LASTEXITCODE -ne 0) {
                throw ("Zig host compile failed for {0} with exit code {1}" -f
                    $variantName, $LASTEXITCODE)
            }
            & $variantExe
            if ($LASTEXITCODE -ne 0) {
                throw ("gyro350 host tests failed for {0} with exit code {1}" -f
                    $variantName, $LASTEXITCODE)
            }
        }
    }

    $authorityProfiles = @(
        @{
            Name = 'A1'
            Defines = @(
                '-DCARTEST_GYRO350_ENABLED=1'
                '-DCARTEST_TRACK_PROFILE=1'
                '-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=5600'
                '-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=4400'
            )
        }
        @{
            Name = 'B2'
            Defines = @(
                '-DCARTEST_GYRO350_ENABLED=1'
                '-DCARTEST_TRACK_PROFILE=2'
                '-DCARTEST_SPEED_PERCENT=100'
                '-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=4800'
                '-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=4000'
            )
        }
        @{
            Name = 'C100'
            Defines = @(
                '-DCARTEST_GYRO350_ENABLED=1'
                '-DCARTEST_TRACK_PROFILE=3'
                '-DCARTEST_SPEED_PERCENT=100'
                '-DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=5600'
                '-DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=4400'
            )
        }
    )
    foreach ($profile in $authorityProfiles) {
        $variantName = $profile.Name + '-authority'
        $variantExe = Join-Path $env:TEMP ("gyro350-authority-tests-" + $variantName + ".exe")
        $compileArgs = @(
            'cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
            '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
            '-I', 'Inc'
        )
        $compileArgs += $profile.Defines
        $compileArgs += @(
            'App\commissioning.c', 'App\gyro350_authority.c', 'App\gyro_ring.c',
            'tests\gyro350\test_authority.c', '-o', $variantExe
        )
        & $ZigPath @compileArgs
        if ($LASTEXITCODE -ne 0) {
            throw ("Zig authority compile failed for {0} with exit code {1}" -f
                $variantName, $LASTEXITCODE)
        }
        & $variantExe
        if ($LASTEXITCODE -ne 0) {
            throw ("gyro350 authority tests failed for {0} with exit code {1}" -f
                $variantName, $LASTEXITCODE)
        }
    }
}
finally {
    Pop-Location
}
