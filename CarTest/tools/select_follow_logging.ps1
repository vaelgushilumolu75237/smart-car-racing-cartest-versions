param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("Log", "NoLog")]
    [string] $Variant,
    [string] $WorkspaceRoot = "",
    [switch] $DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($WorkspaceRoot)) {
    $scriptDirectory = $PSScriptRoot
    if ([string]::IsNullOrWhiteSpace($scriptDirectory)) {
        $scriptFilePath = $MyInvocation.MyCommand.Path
        if ([string]::IsNullOrWhiteSpace($scriptFilePath)) {
            $scriptFilePath = $PSCommandPath
        }
        if ([string]::IsNullOrWhiteSpace($scriptFilePath)) {
            throw "Cannot infer the workspace root because the script path is unavailable; pass -WorkspaceRoot explicitly."
        }
        $scriptDirectory = [System.IO.Path]::GetDirectoryName([System.IO.Path]::GetFullPath($scriptFilePath))
    }
    $WorkspaceRoot = Join-Path $scriptDirectory ".."
}

if (-not (Test-Path -LiteralPath $WorkspaceRoot -PathType Container)) {
    throw "Workspace root directory does not exist: $WorkspaceRoot"
}
try {
    $rootPath = [System.IO.Path]::GetFullPath((Resolve-Path -LiteralPath $WorkspaceRoot -ErrorAction Stop).Path)
}
catch {
    throw "Cannot resolve workspace root '$WorkspaceRoot': $($_.Exception.Message)"
}
$launchPath = Join-Path $rootPath ".vscode\launch.json"
$tasksPath = Join-Path $rootPath ".vscode\tasks.json"
foreach ($requiredPath in @($launchPath, $tasksPath)) {
    if (-not (Test-Path -LiteralPath $requiredPath -PathType Leaf)) {
        throw "Required VS Code file is missing: $requiredPath"
    }
}

$launch = Get-Content -LiteralPath $launchPath -Raw -Encoding UTF8 | ConvertFrom-Json
$tasks = Get-Content -LiteralPath $tasksPath -Raw -Encoding UTF8 | ConvertFrom-Json
if ($null -eq $launch -or
    $null -eq $launch.PSObject.Properties['configurations'] -or
    $launch.configurations -isnot [System.Array]) {
    throw "Expected launch.configurations to be an array; refusing to edit $launchPath"
}

$follow = @($launch.configurations | Where-Object { $_.name -match "DebugFollow" })
if ($follow.Count -ne 1) {
    throw "Expected exactly one DebugFollow launch entry; found $($follow.Count)."
}
$configuration = $follow[0]
if (@($configuration.imagesAndSymbols).Count -ne 1) {
    throw "The Follow launch entry must contain exactly one image/symbol pair."
}

$logLabel = "CarTest: Build DebugFollow (motors enabled; PE7 toggle; straight 50%; hard cap 70%; diff=0; RAM log ON)"
$noLogLabel = "CarTest: Build DebugFollowNoLog (motors enabled; PE7 toggle; straight 50%; hard cap 70%; diff=0; RAM log OFF)"
if ($Variant -eq "Log") {
    $preset = "DebugFollow"
    $name = "CarTest H743 - DebugFollow (motors enabled; PE7 toggle; straight 50%; hard cap 70%; diff=0; RAM log ON)"
    $taskLabel = $logLabel
}
else {
    $preset = "DebugFollowNoLog"
    $name = "CarTest H743 - DebugFollowNoLog (motors enabled; PE7 toggle; straight 50%; hard cap 70%; diff=0; RAM log OFF)"
    $taskLabel = $noLogLabel
}

$currentImage = [string]$configuration.imagesAndSymbols[0].imageFileName
if ($currentImage -notmatch '/build/DebugFollow(?:NoLog)?/CarTest\.elf$') {
    throw "The selected Follow entry points to an unexpected ELF: $currentImage"
}
if (@($tasks.tasks | Where-Object { $_.label -ceq $taskLabel }).Count -ne 1) {
    throw "The matching build task is missing or duplicated: $taskLabel"
}
$selectedTask = @($tasks.tasks | Where-Object { $_.label -ceq $taskLabel })[0]
$presetArgumentIndex = [Array]::IndexOf([string[]]$selectedTask.args, "-Preset")
if ($presetArgumentIndex -lt 0 -or
    $presetArgumentIndex + 1 -ge $selectedTask.args.Count -or
    $selectedTask.args[$presetArgumentIndex + 1] -cne $preset) {
    throw "Build task label does not invoke the matching preset '$preset'."
}

$expectedImage = "`${workspaceFolder}/build/$preset/CarTest.elf"
$expectedTask = Join-Path $rootPath "tools\build.ps1"
if (-not (Test-Path -LiteralPath $expectedTask -PathType Leaf)) {
    throw "The local build script is missing: $expectedTask"
}

Write-Host "Follow logging variant: $Variant"
Write-Host "Launch label: $name"
Write-Host "Build task: $taskLabel"
Write-Host "ELF and symbols: $expectedImage"
if ($DryRun) {
    Write-Host "DryRun: launch.json was not changed."
    return
}

$configuration.name = $name
$configuration.preLaunchTask = $taskLabel
$configuration.imagesAndSymbols[0].imageFileName = $expectedImage
$configuration.imagesAndSymbols[0].symbolFileName = $expectedImage

$stamp = Get-Date -Format "yyyyMMdd-HHmmss-fff"
$backupPath = "$launchPath.bak-$stamp"
Copy-Item -LiteralPath $launchPath -Destination $backupPath
$json = $launch | ConvertTo-Json -Depth 100
[System.IO.File]::WriteAllText(
    $launchPath,
    $json + [Environment]::NewLine,
    (New-Object System.Text.UTF8Encoding($false)))
Write-Host "Updated only the existing DebugFollow launch entry. Backup: $backupPath"
