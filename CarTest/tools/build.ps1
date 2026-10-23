param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("Debug", "DebugSteering", "DebugBench", "DebugFollow", "DebugFollowNoLog", "DebugFollowDiff5", "DebugFollowDiff10", "DebugFollowTurn", "DebugFollowSpeedPI", "DebugFollowTurn65", "DebugFollowIncrementalPI", "DebugFollowIncrementalPIBaseline")]
    [string] $Preset,
    [switch] $Fresh
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$localAppData = $env:LOCALAPPDATA
if ([string]::IsNullOrWhiteSpace($localAppData)) {
    throw "LOCALAPPDATA is not set; cannot locate STM32Cube bundles."
}
$bundleRoot = Join-Path $localAppData "stm32cube\bundles"

function Find-LatestBundle {
    param(
        [Parameter(Mandatory = $true)][string] $BundleName,
        [Parameter(Mandatory = $true)][string[]] $RequiredExecutables
    )

    $componentRoot = Join-Path $bundleRoot $BundleName
    if (-not (Test-Path -LiteralPath $componentRoot -PathType Container)) {
        throw "STM32Cube bundle folder is missing: $componentRoot"
    }

    $candidates = @()
    foreach ($directory in (Get-ChildItem -LiteralPath $componentRoot -Directory)) {
        $versionMatch = [System.Text.RegularExpressions.Regex]::Match(
            $directory.Name,
            '^(?<version>\d+(?:\.\d+){1,3})(?:\+st\.(?<st>\d+))?$'
        )
        if (-not $versionMatch.Success) {
            continue
        }

        $binDirectory = Join-Path $directory.FullName "bin"
        $executables = @{}
        $complete = $true
        foreach ($executable in $RequiredExecutables) {
            $executablePath = Join-Path $binDirectory $executable
            if (-not (Test-Path -LiteralPath $executablePath -PathType Leaf)) {
                $complete = $false
                break
            }
            $executables[$executable] = $executablePath
        }
        if (-not $complete) {
            continue
        }

        $version = [Version]$versionMatch.Groups['version'].Value
        $stRevision = 0
        if ($versionMatch.Groups['st'].Success) {
            $stRevision = [int]$versionMatch.Groups['st'].Value
        }
        $candidates += [PSCustomObject]@{
            Name = $directory.Name
            BinDirectory = $binDirectory
            Executables = $executables
            Major = $version.Major
            Minor = $version.Minor
            Build = $version.Build
            Revision = $version.Revision
            StRevision = $stRevision
        }
    }

    if ($candidates.Count -eq 0) {
        $required = [string]::Join(", ", $RequiredExecutables)
        throw "No complete, supported $BundleName bundle was found under $componentRoot. Expected versioned folders containing: $required"
    }

    $sorted = $candidates | Sort-Object -Property @(
        @{ Expression = 'Major'; Descending = $true },
        @{ Expression = 'Minor'; Descending = $true },
        @{ Expression = 'Build'; Descending = $true },
        @{ Expression = 'Revision'; Descending = $true },
        @{ Expression = 'StRevision'; Descending = $true },
        @{ Expression = 'Name'; Descending = $true }
    )
    return $sorted[0]
}

function Get-ToolVersionLine {
    param(
        [Parameter(Mandatory = $true)][string] $ToolPath,
        [Parameter(Mandatory = $true)][string[]] $Arguments
    )

    $output = @(& $ToolPath @Arguments 2>&1)
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "Version check failed for $ToolPath (exit code $exitCode)."
    }
    $line = $output | Where-Object { -not [string]::IsNullOrWhiteSpace([string]$_) } | Select-Object -First 1
    return [string]$line
}

function Get-CacheValue {
    param([string] $CachePath, [string] $Name)

    if (-not (Test-Path -LiteralPath $CachePath -PathType Leaf)) {
        return $null
    }
    $pattern = '^' + [System.Text.RegularExpressions.Regex]::Escape($Name) + ':[^=]*=(.*)$'
    foreach ($line in (Get-Content -LiteralPath $CachePath -Encoding UTF8)) {
        if ($line -match $pattern) {
            return $Matches[1].Trim()
        }
    }
    return $null
}

function Normalize-PathForCompare {
    param([string] $Path)

    if ([string]::IsNullOrWhiteSpace($Path)) {
        return $null
    }
    $value = $Path.Trim().Trim('"')
    if (-not [System.IO.Path]::IsPathRooted($value)) {
        return $value
    }
    try {
        return [System.IO.Path]::GetFullPath($value).TrimEnd([char[]]@('\', '/'))
    }
    catch {
        return $value
    }
}

function Test-CachedProgramMatches {
    param([string] $CachedValue, [string] $ExpectedPath)

    if ([string]::IsNullOrWhiteSpace($CachedValue)) {
        return $true
    }
    $cached = $CachedValue.Trim().Trim('"')
    $expected = [System.IO.Path]::GetFullPath($ExpectedPath)
    if ([System.IO.Path]::IsPathRooted($cached)) {
        return [string]::Equals(
            (Normalize-PathForCompare $cached),
            (Normalize-PathForCompare $expected),
            [System.StringComparison]::OrdinalIgnoreCase
        )
    }
    if ($cached -notmatch '[/\\]') {
        $cachedName = [System.IO.Path]::GetFileNameWithoutExtension($cached)
        $expectedName = [System.IO.Path]::GetFileNameWithoutExtension($expected)
        return [string]::Equals($cachedName, $expectedName, [System.StringComparison]::OrdinalIgnoreCase)
    }
    return $false
}

$cmakeBundle = Find-LatestBundle -BundleName "cmake" -RequiredExecutables @("cmake.exe")
$ninjaBundle = Find-LatestBundle -BundleName "ninja" -RequiredExecutables @("ninja.exe")
$armBundle = Find-LatestBundle -BundleName "gnu-tools-for-stm32" -RequiredExecutables @(
    "arm-none-eabi-gcc.exe",
    "arm-none-eabi-g++.exe",
    "arm-none-eabi-objcopy.exe",
    "arm-none-eabi-size.exe"
)

$cmakeExe = $cmakeBundle.Executables['cmake.exe']
$ninjaExe = $ninjaBundle.Executables['ninja.exe']
$armGcc = $armBundle.Executables['arm-none-eabi-gcc.exe']
$armGxx = $armBundle.Executables['arm-none-eabi-g++.exe']
$armObjcopy = $armBundle.Executables['arm-none-eabi-objcopy.exe']
$armSize = $armBundle.Executables['arm-none-eabi-size.exe']
$env:PATH = "$($ninjaBundle.BinDirectory);$($armBundle.BinDirectory);$env:PATH"

Write-Host "STM32Cube bundle root: $bundleRoot"
$toolVersions = @(
    [PSCustomObject]@{ Name = 'CMake'; Path = $cmakeExe; Args = @('--version') },
    [PSCustomObject]@{ Name = 'Ninja'; Path = $ninjaExe; Args = @('--version') },
    [PSCustomObject]@{ Name = 'ARM GCC'; Path = $armGcc; Args = @('--version') },
    [PSCustomObject]@{ Name = 'ARM G++'; Path = $armGxx; Args = @('--version') },
    [PSCustomObject]@{ Name = 'ARM objcopy'; Path = $armObjcopy; Args = @('--version') },
    [PSCustomObject]@{ Name = 'ARM size'; Path = $armSize; Args = @('--version') }
)
foreach ($tool in $toolVersions) {
    $versionLine = Get-ToolVersionLine -ToolPath $tool.Path -Arguments $tool.Args
    Write-Host ("{0}: {1}" -f $tool.Name, $tool.Path)
    Write-Host ("  version: {0}" -f $versionLine)
}

$buildDirectory = Join-Path $projectRoot "build\$Preset"
$cachePath = Join-Path $buildDirectory "CMakeCache.txt"
$expectedNinja = [System.IO.Path]::GetFullPath($ninjaExe)
$expectedCompilerByName = @{
    'CMAKE_C_COMPILER' = $armGcc
    'CMAKE_CXX_COMPILER' = $armGxx
    'CMAKE_ASM_COMPILER' = $armGcc
}

if ((Test-Path -LiteralPath $cachePath -PathType Leaf) -and -not $Fresh) {
    $mismatches = @()
    $cachedSource = Get-CacheValue -CachePath $cachePath -Name 'CMAKE_HOME_DIRECTORY'
    if (-not [string]::IsNullOrWhiteSpace($cachedSource) -and
        -not [string]::Equals((Normalize-PathForCompare $cachedSource), (Normalize-PathForCompare $projectRoot), [System.StringComparison]::OrdinalIgnoreCase)) {
        $mismatches += "source: cache=$cachedSource; current=$projectRoot"
    }

    foreach ($compilerName in $expectedCompilerByName.Keys) {
        $cachedCompiler = Get-CacheValue -CachePath $cachePath -Name $compilerName
        if (-not (Test-CachedProgramMatches -CachedValue $cachedCompiler -ExpectedPath $expectedCompilerByName[$compilerName])) {
            $mismatches += "$compilerName`: cache=$cachedCompiler; current=$($expectedCompilerByName[$compilerName])"
        }
    }

    $compilerMetadataRoot = Join-Path $buildDirectory 'CMakeFiles'
    if (Test-Path -LiteralPath $compilerMetadataRoot -PathType Container) {
        foreach ($versionDirectory in (Get-ChildItem -LiteralPath $compilerMetadataRoot -Directory)) {
            foreach ($metadataName in @('CMakeCCompiler.cmake', 'CMakeCXXCompiler.cmake', 'CMakeASMCompiler.cmake')) {
                $metadataPath = Join-Path $versionDirectory.FullName $metadataName
                if (-not (Test-Path -LiteralPath $metadataPath -PathType Leaf)) {
                    continue
                }
                $metadataText = Get-Content -LiteralPath $metadataPath -Raw -Encoding UTF8
                if ($metadataText -match 'set\(CMAKE_(?:C|CXX|ASM)_COMPILER "([^"]+)"\)') {
                    $cachedCompiler = $Matches[1]
                    if ($metadataName -eq 'CMakeCXXCompiler.cmake') {
                        $expectedCompiler = $armGxx
                        $compilerName = 'CMAKE_CXX_COMPILER'
                    }
                    else {
                        $expectedCompiler = $armGcc
                        if ($metadataName -eq 'CMakeASMCompiler.cmake') { $compilerName = 'CMAKE_ASM_COMPILER' }
                        else { $compilerName = 'CMAKE_C_COMPILER' }
                    }
                    if (-not (Test-CachedProgramMatches -CachedValue $cachedCompiler -ExpectedPath $expectedCompiler)) {
                        $mismatches += "$compilerName metadata: cache=$cachedCompiler; current=$expectedCompiler"
                    }
                }
            }
        }
    }

    $cachedNinja = Get-CacheValue -CachePath $cachePath -Name 'CMAKE_MAKE_PROGRAM'
    if (-not (Test-CachedProgramMatches -CachedValue $cachedNinja -ExpectedPath $expectedNinja)) {
        $mismatches += "Ninja: cache=$cachedNinja; current=$expectedNinja"
    }

    if ($mismatches.Count -gt 0) {
        $details = [string]::Join([Environment]::NewLine, ($mismatches | Select-Object -Unique | ForEach-Object { "  - $_" }))
        throw "Existing CMake cache does not match this source/toolchain:`n$details`nRun this command once to reconfigure safely: .\tools\build.ps1 -Preset $Preset -Fresh"
    }
}

$expectedElf = Join-Path $projectRoot "build\$Preset\CarTest.elf"
Push-Location $projectRoot
try {
    if ($Fresh) {
        Write-Host "Freshly configuring CarTest preset '$Preset' with CMake --fresh (no build directory is recursively deleted)..."
        & $cmakeExe --fresh --preset $Preset
    }
    else {
        Write-Host "Configuring CarTest preset '$Preset'..."
        & $cmakeExe --preset $Preset
    }
    if ($LASTEXITCODE -ne 0) {
        throw "CMake configure failed with exit code $LASTEXITCODE."
    }

    $expectedProfileFlags = @{
        'Debug' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'OFF'; CARTEST_ENABLE_FOLLOW_TURN = 'OFF'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'OFF'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '0'; CARTEST_VISION_HOLD_MS = '3000'; CARTEST_VISION_HOLD_RAMP_MS = '300' }
        'DebugSteering' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'OFF'; CARTEST_ENABLE_FOLLOW_TURN = 'OFF'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'ON'; CARTEST_HARDWARE_ACCEPTED = 'OFF'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '0'; CARTEST_VISION_HOLD_MS = '3000'; CARTEST_VISION_HOLD_RAMP_MS = '300' }
        'DebugBench' = @{ CARTEST_ENABLE_BENCH = 'ON'; CARTEST_ENABLE_FOLLOW = 'OFF'; CARTEST_ENABLE_FOLLOW_TURN = 'OFF'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'OFF'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '0'; CARTEST_VISION_HOLD_MS = '3000'; CARTEST_VISION_HOLD_RAMP_MS = '300' }
        'DebugFollow' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'OFF'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '0'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
        'DebugFollowNoLog' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'OFF'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'OFF'; CARTEST_FOLLOW_DIFF_PERMILLE = '0'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
        'DebugFollowDiff5' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'OFF'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '50'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
        'DebugFollowDiff10' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'OFF'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '100'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
        'DebugFollowTurn' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'ON'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '200'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
        'DebugFollowSpeedPI' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'ON'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'ON'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '200'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
        'DebugFollowTurn65' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'ON'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'OFF'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'OFF'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '150'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
        'DebugFollowIncrementalPI' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'ON'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'ON'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'ON'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '150'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
        'DebugFollowIncrementalPIBaseline' = @{ CARTEST_ENABLE_BENCH = 'OFF'; CARTEST_ENABLE_FOLLOW = 'ON'; CARTEST_ENABLE_FOLLOW_TURN = 'ON'; CARTEST_ENABLE_FOLLOW_SPEED_PI = 'ON'; CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI = 'ON'; CARTEST_ENABLE_STEERING = 'OFF'; CARTEST_HARDWARE_ACCEPTED = 'ON'; CARTEST_RUNTIME_LOG_ENABLED = 'ON'; CARTEST_FOLLOW_DIFF_PERMILLE = '150'; CARTEST_VISION_HOLD_MS = '2000'; CARTEST_VISION_HOLD_RAMP_MS = '200' }
    }
    $configuredCache = Join-Path $buildDirectory 'CMakeCache.txt'
    foreach ($flagName in $expectedProfileFlags[$Preset].Keys) {
        $actualFlag = Get-CacheValue -CachePath $configuredCache -Name $flagName
        if ($actualFlag -ne $expectedProfileFlags[$Preset][$flagName]) {
            throw "Configured profile flag mismatch for $Preset`: $flagName=$actualFlag"
        }
    }
    $expectedTurnProfile = if ($Preset -in @('DebugFollowTurn', 'DebugFollowSpeedPI', 'DebugFollowTurn65', 'DebugFollowIncrementalPI', 'DebugFollowIncrementalPIBaseline')) { 'ON' } else { 'OFF' }
    $actualTurnProfile = Get-CacheValue -CachePath $configuredCache -Name 'CARTEST_ENABLE_FOLLOW_TURN'
    if ($actualTurnProfile -ne $expectedTurnProfile) {
        throw "Configured Follow turn profile mismatch for $Preset`: CARTEST_ENABLE_FOLLOW_TURN=$actualTurnProfile"
    }
    $expectedAckermannOption = if ($Preset -eq 'DebugFollowIncrementalPIBaseline') { 'OFF' } else { 'ON' }
    $actualAckermannOption = Get-CacheValue -CachePath $configuredCache -Name 'CARTEST_ENABLE_ACKERMANN_COORDINATION'
    if ($actualAckermannOption -ne $expectedAckermannOption) {
        throw "Configured Ackermann coordination mismatch for $Preset`: CARTEST_ENABLE_ACKERMANN_COORDINATION=$actualAckermannOption"
    }
    $compileCommandsPath = Join-Path $buildDirectory 'compile_commands.json'
    if (-not (Test-Path -LiteralPath $compileCommandsPath -PathType Leaf)) {
        throw "CMake compile commands are missing; cannot verify the effective profile defines: $compileCommandsPath"
    }
    $compileCommands = Get-Content -LiteralPath $compileCommandsPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $mainCompile = $compileCommands | Where-Object { $_.file -match '[/\\]Src[/\\]main\.c$' } | Select-Object -First 1
    if (-not $mainCompile -or [string]::IsNullOrWhiteSpace([string]$mainCompile.command)) {
        throw "CMake compile command for Src/main.c is missing from $compileCommandsPath"
    }
    $expectedTurnBit = if ($expectedTurnProfile -eq 'ON') { '1' } else { '0' }
    $expectedSpeedPiBit = if ($expectedProfileFlags[$Preset].CARTEST_ENABLE_FOLLOW_SPEED_PI -eq 'ON') { '1' } else { '0' }
    $expectedIncrementalPiBit = if ($expectedProfileFlags[$Preset].CARTEST_ENABLE_FOLLOW_INCREMENTAL_PI -eq 'ON') { '1' } else { '0' }
    $expectedDiff = $expectedProfileFlags[$Preset].CARTEST_FOLLOW_DIFF_PERMILLE
    $expectedFollowProfile = $expectedProfileFlags[$Preset].CARTEST_ENABLE_FOLLOW -eq 'ON'
    $expectedTurnStep = if ($expectedFollowProfile) { '40' } else { '10' }
    $expectedServoStep = if ($Preset -eq 'DebugFollowIncrementalPI') { '160' } elseif ($expectedFollowProfile) { '120' } else { '40' }
    $expectedAckermannBit = if ($Preset -eq 'DebugFollowIncrementalPI') { '1' } else { '0' }
    $expectedNormalLimit = if ($expectedFollowProfile) { '1000' } else { '600' }
    $expectedDefines = @{
        CARTEST_FOLLOW_DIFF_PERMILLE = $expectedDiff
        CARTEST_FOLLOW_INNER_DIFF_PERMILLE = $expectedDiff
        CARTEST_FOLLOW_TURN_PROFILE_COMPILED = $expectedTurnBit
        CARTEST_FOLLOW_SPEED_PI_ENABLED = $expectedSpeedPiBit
        CARTEST_FOLLOW_INCREMENTAL_PI_ENABLED = $expectedIncrementalPiBit
        CARTEST_ACKERMANN_COORDINATION_ENABLED = $expectedAckermannBit
        CARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE = '10'
        CARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE = $expectedTurnStep
        CARTEST_FOLLOW_SERVO_SLEW_STEP_US = $expectedServoStep
        CARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE = $expectedNormalLimit
        CARTEST_FOLLOW_HARD_LIMIT_PERMILLE = '1000'
        CARTEST_VISION_HOLD_MS = $expectedProfileFlags[$Preset].CARTEST_VISION_HOLD_MS
        CARTEST_VISION_HOLD_RAMP_MS = $expectedProfileFlags[$Preset].CARTEST_VISION_HOLD_RAMP_MS
    }
    foreach ($defineName in $expectedDefines.Keys) {
        $expectedDefine = "-D$defineName=$($expectedDefines[$defineName])"
        if (-not ([string]$mainCompile.command).Contains($expectedDefine)) {
            throw "Effective compile profile for $Preset is missing '$expectedDefine' in the Src/main.c compile command."
        }
    }
    Write-Host "Verified $($expectedDefines.Count) effective compile definitions in Src/main.c for '$Preset'."
    Write-Host "Building target CarTest with preset '$Preset'..."
    & $cmakeExe --build --preset $Preset --target CarTest
    if ($LASTEXITCODE -ne 0) {
        throw "CarTest build failed with exit code $LASTEXITCODE."
    }

    if (-not (Test-Path -LiteralPath $expectedElf -PathType Leaf)) {
        throw "The expected CarTest ELF was not produced: $expectedElf"
    }

    $resolvedElf = (Resolve-Path -LiteralPath $expectedElf).Path
    if (-not [string]::Equals($resolvedElf, [System.IO.Path]::GetFullPath($expectedElf), [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "ELF path does not match the selected CarTest build directory: $resolvedElf"
    }

    Write-Host "CarTest ELF: $resolvedElf"
}
finally {
    Pop-Location
}
