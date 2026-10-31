[CmdletBinding()]
param(
    [switch]$VerifyOnly,
    [ValidatePattern('^[A-Za-z]$')]
    [string]$DriveLetter = 'P',
    [ValidatePattern('^[A-Za-z]$')]
    [string]$BuildDriveLetter = 'W',
    [string]$RuntimeRoot
)

$ErrorActionPreference = 'Stop'

$runnerDirectory = [IO.Path]::GetFullPath($PSScriptRoot)
$packageRoot = [IO.Path]::GetFullPath((Join-Path $runnerDirectory '..'))
$gitRoot = Join-Path $packageRoot 'tools\git'
$bashPath = Join-Path $gitRoot 'usr\bin\bash.exe'
$cygpathPath = Join-Path $gitRoot 'usr\bin\cygpath.exe'
$substPath = Join-Path $env:SystemRoot 'System32\subst.exe'
$shellRunner = Join-Path $runnerDirectory 'build-openmv4p-app-only.sh'
$v25RuntimeRoot = Join-Path $packageRoot 'project\deliverables\TrackProfiles_RingJoint_v25_20261029\OpenMV'
$selectedRuntimeRoot = if ($RuntimeRoot) { [IO.Path]::GetFullPath($RuntimeRoot) } elseif (
    Test-Path -LiteralPath (Join-Path $v25RuntimeRoot 'PACKAGE_MANIFEST.json')) {
    $v25RuntimeRoot
} else {
    Join-Path $packageRoot 'project\deliverables\TrackProfiles_Ring45_USB60_v16_20261027\OpenMV'
}
$packageDriveLetter = $DriveLetter.ToUpperInvariant()
$packageDrive = '{0}:' -f $packageDriveLetter
$packageDriveRoot = '{0}:\' -f $packageDriveLetter
$buildDriveLetter = $BuildDriveLetter.ToUpperInvariant()
if ($packageDriveLetter -in @('Q', 'R') -or $buildDriveLetter -in @('Q', 'R')) {
    throw 'Q: and R: are reserved; choose different temporary package and build drives.'
}
if ($packageDriveLetter -eq $buildDriveLetter) {
    throw 'Package and build drive letters must differ.'
}
$buildDrive = '{0}:' -f $buildDriveLetter
$buildDriveRoot = '{0}:\' -f $buildDriveLetter
$workRoot = Join-Path $runnerDirectory 'work'

$requiredPaths = @(
    (Join-Path $packageRoot 'source\openmv\.git'),
    (Join-Path $packageRoot 'source\openmv\Makefile'),
    (Join-Path $packageRoot 'tools\arm\bin\arm-none-eabi-gcc.exe'),
    (Join-Path $packageRoot 'tools\build-tools\bin\make.exe'),
    (Join-Path $packageRoot 'tools\python\python.exe'),
    (Join-Path $packageRoot 'tools\mpy-cross-1.28.0\mpy-cross.exe'),
    (Join-Path $packageRoot 'tools\sdk-marker\sdk.version'),
    (Join-Path $packageRoot 'project\tools\vendor-patches\openmv-v5.0.1\build-openmv4p-ring45-native-clean.sh'),
    (Join-Path $selectedRuntimeRoot 'PACKAGE_MANIFEST.json'),
    $bashPath,
    $cygpathPath,
    $shellRunner,
    (Join-Path $runnerDirectory 'python3')
)

$missing = @($requiredPaths | Where-Object { -not (Test-Path -LiteralPath $_) })
if ($missing.Count -gt 0) {
    $missingText = $missing -join [Environment]::NewLine
    throw "The offline OpenMV build package is incomplete. Missing:`n$missingText"
}

if (-not (Test-Path -LiteralPath (Join-Path $packageRoot 'source\openmv\lib\micropython\.git'))) {
    throw 'The packaged MicroPython submodule metadata is missing.'
}

$preexistingPsDrive = Get-PSDrive -Name $packageDriveLetter -ErrorAction SilentlyContinue
$preexistingNativeDrive = [System.IO.DriveInfo]::GetDrives() |
    Where-Object { $_.Name -ieq $packageDriveRoot }
$preexistingSubst = @(& $substPath) |
    Where-Object { $_.TrimStart().StartsWith(($packageDriveRoot + ':'), [StringComparison]::OrdinalIgnoreCase) }
if ($preexistingPsDrive -or $preexistingNativeDrive -or $preexistingSubst) {
    throw "$packageDrive is already in use; choose an unused drive with -DriveLetter."
}

$preexistingBuildPsDrive = Get-PSDrive -Name $buildDriveLetter -ErrorAction SilentlyContinue
$preexistingBuildNativeDrive = [System.IO.DriveInfo]::GetDrives() |
    Where-Object { $_.Name -ieq $buildDriveRoot }
$preexistingBuildSubst = @(& $substPath) |
    Where-Object { $_.TrimStart().StartsWith(($buildDriveRoot + ':'), [StringComparison]::OrdinalIgnoreCase) }
if ($preexistingBuildPsDrive -or $preexistingBuildNativeDrive -or $preexistingBuildSubst) {
    throw "$buildDrive is already in use; choose an unused drive with -BuildDriveLetter."
}

if (-not (Test-Path -LiteralPath $workRoot)) {
    New-Item -ItemType Directory -Path $workRoot | Out-Null
}
$buildRunId = '{0}-{1}-{2}' -f (Get-Date -Format 'yyMMddHHmmss'), $PID, ([Guid]::NewGuid().ToString('N').Substring(0, 6))
$buildDirectory = Join-Path $workRoot "b_$buildRunId"
while (Test-Path -LiteralPath $buildDirectory) {
    $buildRunId = '{0}-{1}-{2}' -f (Get-Date -Format 'yyMMddHHmmss'), $PID, ([Guid]::NewGuid().ToString('N').Substring(0, 6))
    $buildDirectory = Join-Path $workRoot "b_$buildRunId"
}
New-Item -ItemType Directory -Path $buildDirectory | Out-Null
$buildDirectoryCreated = $true

$originalPath = $env:PATH
$gitPathEntries = @(
    (Join-Path $gitRoot 'usr\bin'),
    (Join-Path $gitRoot 'mingw64\bin'),
    (Join-Path $gitRoot 'cmd'),
    (Join-Path $gitRoot 'bin'),
    (Join-Path $env:SystemRoot 'System32')
)
$env:PATH = ($gitPathEntries -join ';')
$packageMappingCreated = $false
$buildMappingCreated = $false
$previousBuildRootPosix = $env:OPENMV_BUILD_ROOT_POSIX
$previousPackageRootNative = $env:OPENMV_PACKAGE_ROOT_NATIVE
$previousBuildRootNative = $env:OPENMV_BUILD_ROOT_NATIVE
$previousBuildDirNative = $env:OPENMV_BUILD_DIR_NATIVE
$previousPackageDrive = $env:OPENMV_PACKAGE_DRIVE
$previousBuildDrive = $env:OPENMV_BUILD_DRIVE
$previousRunId = $env:OPENMV_RUN_ID
$previousRuntimeRoot = $env:OPENMV_RUNTIME_ROOT_NATIVE

try {
    & $substPath $packageDrive $packageRoot
    if ($LASTEXITCODE -ne 0) {
        throw "Could not create temporary $packageDrive package mapping."
    }
    $packageMappingCreated = $true

    & $substPath $buildDrive $buildDirectory
    if ($LASTEXITCODE -ne 0) {
        throw "Could not create temporary $buildDrive build-work mapping."
    }
    $buildMappingCreated = $true

    $shortShellRunner = Join-Path $packageDriveRoot 'runner\build-openmv4p-app-only.sh'
    $posixRunnerOutput = @(& $cygpathPath -u $shortShellRunner)
    if ($LASTEXITCODE -ne 0 -or $posixRunnerOutput.Count -eq 0) {
        throw 'The packaged cygpath.exe could not convert the runner path.'
    }
    $posixRunner = ($posixRunnerOutput -join "`n").Trim()
    $posixBuildRootOutput = @(& $cygpathPath -u $buildDriveRoot)
    if ($LASTEXITCODE -ne 0 -or $posixBuildRootOutput.Count -eq 0) {
        throw 'The packaged cygpath.exe could not convert the temporary build-work path.'
    }
    $env:OPENMV_BUILD_ROOT_POSIX = ($posixBuildRootOutput -join "`n").Trim().TrimEnd('/')
    $env:OPENMV_PACKAGE_ROOT_NATIVE = $packageRoot
    $env:OPENMV_BUILD_ROOT_NATIVE = $buildDirectory
    $env:OPENMV_BUILD_DIR_NATIVE = $buildDirectory
    $env:OPENMV_PACKAGE_DRIVE = $packageDriveLetter
    $env:OPENMV_BUILD_DRIVE = $buildDriveLetter
    $env:OPENMV_RUN_ID = $buildRunId
    $env:OPENMV_RUNTIME_ROOT_NATIVE = $selectedRuntimeRoot

    $bashArguments = @('--noprofile', '--norc', $posixRunner)
    if ($VerifyOnly) {
        $bashArguments += '--verify-only'
    }

    Write-Host 'OPENMV_BUILD_TARGET=OPENMV4P / STM32H743xx / app-only'
    Write-Host 'OFFLINE_DEPENDENCIES=package-local; HARDWARE_FLASH_UPLOAD=not-performed'
    Write-Host "PACKAGE_ROOT=$packageRoot"
    Write-Host "RUNTIME_ROOT=$selectedRuntimeRoot"
    Write-Host "TEMP_PACKAGE_DRIVE=$packageDrive (removed after this invocation)"
    Write-Host "TEMP_BUILD_DRIVE=$buildDrive (removed after this invocation)"

    & $bashPath @bashArguments
    $runnerExitCode = $LASTEXITCODE
}
finally {
    if ($buildMappingCreated) {
        try {
            $substLines = @(& $substPath)
            $mappingLine = $substLines |
                Where-Object { $_.TrimStart().StartsWith(($buildDriveRoot + ':'), [StringComparison]::OrdinalIgnoreCase) } |
                Select-Object -First 1
            $mappedTarget = $null
            if ($mappingLine) {
                $mappingParts = ([string]$mappingLine) -split '=>', 2
                if ($mappingParts.Count -eq 2) {
                    $mappedTarget = [IO.Path]::GetFullPath($mappingParts[1].Trim()).TrimEnd('\')
                }
            }
            if ($mappedTarget -and $mappedTarget -ieq $buildDirectory.TrimEnd('\')) {
                & $substPath $buildDrive '/D'
                if ($LASTEXITCODE -ne 0) {
                    Write-Warning "Could not remove temporary $buildDrive mapping."
                }
            }
            else {
                Write-Warning "Leaving $buildDrive untouched because its current target no longer matches this fresh build directory."
            }
        }
        catch {
            Write-Warning "Could not verify or remove temporary $buildDrive mapping: $_"
        }
    }
    if ($packageMappingCreated) {
        try {
            $substLines = @(& $substPath)
            $mappingLine = $substLines |
                Where-Object { $_.TrimStart().StartsWith(($packageDriveRoot + ':'), [StringComparison]::OrdinalIgnoreCase) } |
                Select-Object -First 1
            $mappedTarget = $null
            if ($mappingLine) {
                $mappingParts = ([string]$mappingLine) -split '=>', 2
                if ($mappingParts.Count -eq 2) {
                    $mappedTarget = [IO.Path]::GetFullPath($mappingParts[1].Trim()).TrimEnd('\')
                }
            }
            if ($mappedTarget -and $mappedTarget -ieq $packageRoot.TrimEnd('\')) {
                & $substPath $packageDrive '/D'
                if ($LASTEXITCODE -ne 0) {
                    Write-Warning "Could not remove temporary $packageDrive mapping."
                }
            }
            else {
                Write-Warning "Leaving $packageDrive untouched because its current target no longer matches this package."
            }
        }
        catch {
            Write-Warning "Could not verify or remove temporary $packageDrive mapping: $_"
        }
    }
    $env:OPENMV_BUILD_ROOT_POSIX = $previousBuildRootPosix
    $env:OPENMV_PACKAGE_ROOT_NATIVE = $previousPackageRootNative
    $env:OPENMV_BUILD_ROOT_NATIVE = $previousBuildRootNative
    $env:OPENMV_BUILD_DIR_NATIVE = $previousBuildDirNative
    $env:OPENMV_PACKAGE_DRIVE = $previousPackageDrive
    $env:OPENMV_BUILD_DRIVE = $previousBuildDrive
    $env:OPENMV_RUN_ID = $previousRunId
    $env:OPENMV_RUNTIME_ROOT_NATIVE = $previousRuntimeRoot
    $env:PATH = $originalPath
    if ($VerifyOnly -and $buildDirectoryCreated -and (Test-Path -LiteralPath $buildDirectory)) {
        $remaining = @(Get-ChildItem -LiteralPath $buildDirectory -Force)
        if ($remaining.Count -eq 0) {
            Remove-Item -LiteralPath $buildDirectory
        }
        else {
            Write-Warning "Keeping non-empty verify-only directory: $buildDirectory"
        }
    }
}

if ($runnerExitCode -ne 0) {
    exit $runnerExitCode
}

exit 0
