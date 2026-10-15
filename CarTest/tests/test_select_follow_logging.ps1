param(
    [string] $EvidenceRoot = (Join-Path $PSScriptRoot "..\test-data\runtime-log-ab-20261012\selector")
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$runRoot = Join-Path $EvidenceRoot "workspace-$(Get-Date -Format 'yyyyMMdd-HHmmss-fff')"
$launchSource = Join-Path $projectRoot ".vscode\launch.json"
$tasksSource = Join-Path $projectRoot ".vscode\tasks.json"
$selectorSource = Join-Path $projectRoot "tools\select_follow_logging.ps1"
$buildSource = Join-Path $projectRoot "tools\build.ps1"
$launchTarget = Join-Path $runRoot ".vscode\launch.json"
$tasksTarget = Join-Path $runRoot ".vscode\tasks.json"
$selectorTarget = Join-Path $runRoot "tools\select_follow_logging.ps1"

New-Item -ItemType Directory -Force -Path (Join-Path $runRoot ".vscode") | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $runRoot "tools") | Out-Null
Copy-Item -LiteralPath $launchSource -Destination $launchTarget
Copy-Item -LiteralPath $tasksSource -Destination $tasksTarget
Copy-Item -LiteralPath $selectorSource -Destination $selectorTarget
Copy-Item -LiteralPath $buildSource -Destination (Join-Path $runRoot "tools\build.ps1")

function Read-Launch {
    return Get-Content -LiteralPath $launchTarget -Raw -Encoding UTF8 | ConvertFrom-Json
}

function Get-Follow {
    param($Launch)
    $found = @($Launch.configurations | Where-Object { $_.name -match "DebugFollow" })
    if ($found.Count -ne 1) { throw "Expected exactly one Follow launch entry." }
    return $found[0]
}

function Assert-Variant {
    param([string] $Variant, [string] $Preset, [string] $TaskLabel)
    $launch = Read-Launch
    if (@($launch.configurations).Count -ne $menuCount) {
        throw "Selector changed the launch menu count."
    }
    $selected = Get-Follow $launch
    $expectedPath = "`${workspaceFolder}/build/$Preset/CarTest.elf"
    $expectedLogLabel = if ($Variant -eq "Log") { "RAM log ON" } else { "RAM log OFF" }
    if ($selected.imagesAndSymbols[0].imageFileName -cne $expectedPath -or
        $selected.imagesAndSymbols[0].symbolFileName -cne $expectedPath -or
        $selected.preLaunchTask -cne $TaskLabel -or
        $selected.name -notmatch [regex]::Escape($expectedLogLabel)) {
        throw "Selected launch fields do not consistently target $Variant / $Preset."
    }
    $tasks = Get-Content -LiteralPath $tasksTarget -Raw -Encoding UTF8 | ConvertFrom-Json
    $task = @($tasks.tasks | Where-Object { $_.label -ceq $TaskLabel })
    if ($task.Count -ne 1) { throw "Expected one matching build task for $Variant." }
    $index = [Array]::IndexOf([string[]]$task[0].args, "-Preset")
    if ($index -lt 0 -or $task[0].args[$index + 1] -cne $Preset) {
        throw "Build task for $Variant points at the wrong preset."
    }
}

$initial = Read-Launch
$menuCount = @($initial.configurations).Count
$otherBefore = @($initial.configurations | Where-Object { $_.name -notmatch "DebugFollow" } |
    ForEach-Object { $_ | ConvertTo-Json -Depth 100 -Compress })
$followBefore = Get-Follow $initial
$selectedPreservedBefore = $followBefore | ConvertTo-Json -Depth 100 | ConvertFrom-Json
$tasksHashBefore = (Get-FileHash -LiteralPath $tasksTarget -Algorithm SHA256).Hash
$launchHashBeforeDryRun = (Get-FileHash -LiteralPath $launchTarget -Algorithm SHA256).Hash

& $selectorTarget -Variant NoLog -DryRun
if ((Get-FileHash -LiteralPath $launchTarget -Algorithm SHA256).Hash -cne $launchHashBeforeDryRun) {
    throw "DryRun changed launch.json."
}

$logLabel = "CarTest: Build DebugFollow (motors enabled; PE7 toggle; straight 50%; hard cap 70%; diff=0; RAM log ON)"
$noLogLabel = "CarTest: Build DebugFollowNoLog (motors enabled; PE7 toggle; straight 50%; hard cap 70%; diff=0; RAM log OFF)"
& $selectorTarget -WorkspaceRoot $runRoot -Variant Log
Assert-Variant -Variant "Log" -Preset "DebugFollow" -TaskLabel $logLabel
& $selectorTarget -WorkspaceRoot $runRoot -Variant NoLog
Assert-Variant -Variant "NoLog" -Preset "DebugFollowNoLog" -TaskLabel $noLogLabel
$backupCount = @(Get-ChildItem -LiteralPath (Split-Path -Parent $launchTarget) -Filter "launch.json.bak-*" -File).Count
if ($backupCount -ne 2) { throw "Log/NoLog selections did not preserve launch.json backups." }

$selectedAfter = Get-Follow (Read-Launch)
foreach ($property in @("type", "request", "cwd", "serverInterface", "serverInterfaceFrequency", "serverReset", "liveWatch")) {
    $beforeValue = $selectedPreservedBefore.$property | ConvertTo-Json -Depth 100 -Compress
    $afterValue = $selectedAfter.$property | ConvertTo-Json -Depth 100 -Compress
    if ($beforeValue -cne $afterValue) { throw "Selector changed unrelated Follow property '$property'." }
}
$otherAfter = @((Read-Launch).configurations | Where-Object { $_.name -notmatch "DebugFollow" } |
    ForEach-Object { $_ | ConvertTo-Json -Depth 100 -Compress })
if ((ConvertTo-Json $otherBefore -Compress) -cne (ConvertTo-Json $otherAfter -Compress)) {
    throw "Selector changed one of the three non-Follow launch entries."
}

& $selectorTarget -WorkspaceRoot $runRoot -Variant Log
Assert-Variant -Variant "Log" -Preset "DebugFollow" -TaskLabel $logLabel
if ((Get-FileHash -LiteralPath $tasksTarget -Algorithm SHA256).Hash -cne $tasksHashBefore) {
    throw "Selector changed tasks.json."
}

Write-Host "SELECTOR_TEST_PASS launch_entries=$menuCount sequence=Log,NoLog,Log dry_run=PASS backups=3 other_launch_entries=unchanged tasks=unchanged"
Write-Host "Temporary workspace: $runRoot"

