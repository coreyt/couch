# collect_sofa_deps_win.ps1
# Collects SOFA DLL dependencies for the SofaAnkleBridge native plugin.
# Usage: .\collect_sofa_deps_win.ps1 -SofaRoot C:\sofa\SOFA_v24.06.00_Win64 -DllPath .\build-win\Release\SofaAnkleBridge.dll -OutputDir .\sofa_deps

param(
    [Parameter(Mandatory=$true)]
    [string]$SofaRoot,

    [Parameter(Mandatory=$true)]
    [string]$DllPath,

    [Parameter(Mandatory=$true)]
    [string]$OutputDir
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path $DllPath)) {
    Write-Error "DLL not found: $DllPath"
    exit 1
}

if (-not (Test-Path $SofaRoot)) {
    Write-Error "SOFA_ROOT not found: $SofaRoot"
    exit 1
}

# Create output directory
if (-not (Test-Path $OutputDir)) {
    New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
}

$sofaBinDir = Join-Path $SofaRoot "bin"
$sofaLibDir = Join-Path $SofaRoot "lib"

# Find dumpbin.exe from Visual Studio
$dumpbin = Get-Command dumpbin -ErrorAction SilentlyContinue
if (-not $dumpbin) {
    # Try to find via vswhere
    $vsPath = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath 2>$null
    if ($vsPath) {
        $dumpbinPath = Get-ChildItem "$vsPath\VC\Tools\MSVC\*\bin\Hostx64\x64\dumpbin.exe" -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($dumpbinPath) {
            $dumpbin = $dumpbinPath.FullName
        }
    }
}

if (-not $dumpbin) {
    Write-Warning "dumpbin.exe not found. Using file-name heuristic instead."
    # Fallback: just copy all DLLs from SOFA bin that start with Sofa
    $sofaDlls = Get-ChildItem "$sofaBinDir\*.dll" -ErrorAction SilentlyContinue
    foreach ($dll in $sofaDlls) {
        $dest = Join-Path $OutputDir $dll.Name
        Copy-Item $dll.FullName $dest -Force
        Write-Host "  Copied: $($dll.Name)"
    }
    Write-Host "`nCopied $($sofaDlls.Count) DLLs to $OutputDir (heuristic mode)"
    exit 0
}

# Walk dependency tree
$visited = @{}
$queue = @($DllPath)

while ($queue.Count -gt 0) {
    $current = $queue[0]
    $queue = $queue[1..$queue.Count]

    $currentName = [System.IO.Path]::GetFileName($current)
    if ($visited.ContainsKey($currentName.ToLower())) { continue }
    $visited[$currentName.ToLower()] = $true

    Write-Host "Scanning: $currentName"

    # Get dependencies via dumpbin
    $output = & $dumpbin /dependents $current 2>&1
    $deps = $output | Where-Object { $_ -match '^\s+\S+\.dll$' } |
            ForEach-Object { $_.Trim() }

    foreach ($dep in $deps) {
        $depLower = $dep.ToLower()
        if ($visited.ContainsKey($depLower)) { continue }

        # Check if this DLL exists in SOFA bin or lib
        $depPath = $null
        $binPath = Join-Path $sofaBinDir $dep
        $libPath = Join-Path $sofaLibDir $dep

        if (Test-Path $binPath) {
            $depPath = $binPath
        } elseif (Test-Path $libPath) {
            $depPath = $libPath
        }

        if ($depPath) {
            $dest = Join-Path $OutputDir $dep
            Copy-Item $depPath $dest -Force
            Write-Host "  Copied: $dep"
            $queue += $depPath
        }
    }
}

$copiedCount = (Get-ChildItem "$OutputDir\*.dll" -ErrorAction SilentlyContinue).Count
Write-Host "`nCollected $copiedCount SOFA DLL dependencies to $OutputDir"
