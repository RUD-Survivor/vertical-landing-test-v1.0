# build_find_stale.ps1 — decide which .cpp units need recompile
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File build_find_stale.ps1 `
#     -ObjDir build\obj\debug -SourcesFile build\obj\debug\sources.lst [-Force]
#
# Rules (exit writes space-separated stale .cpp paths to stdout):
#   1) -Force, or .obj missing                         -> rebuild
#   2) .cpp newer than .obj                            -> rebuild
#   3) any .h/.hpp/.inl/.hxx under the unit's roots
#      newer than .obj                                 -> rebuild
#      - normal units: roots = directory of the .cpp (recursive)
#      - main.cpp:     roots = src\ + vendor\imgui\
#        (main pulls in most header-only Vulkan/render code)
param(
    [Parameter(Mandatory = $true)][string]$ObjDir,
    [Parameter(Mandatory = $true)][string]$SourcesFile,
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $SourcesFile)) {
    Write-Error "SourcesFile not found: $SourcesFile"
    exit 1
}

$sources = Get-Content -LiteralPath $SourcesFile |
    ForEach-Object { $_.Trim() } |
    Where-Object { $_ -ne '' }

$headerExt = @{ '.h' = $true; '.hpp' = $true; '.inl' = $true; '.hxx' = $true; '.glsl' = $true }
$stale = New-Object System.Collections.Generic.List[string]

function Test-AnyHeaderNewer([string[]]$Roots, [datetime]$ObjTime) {
    foreach ($root in $Roots) {
        if (-not (Test-Path -LiteralPath $root)) { continue }
        $hit = Get-ChildItem -LiteralPath $root -Recurse -File -ErrorAction SilentlyContinue |
            Where-Object { $headerExt.ContainsKey($_.Extension.ToLowerInvariant()) -and $_.LastWriteTime -gt $ObjTime } |
            Select-Object -First 1
        if ($null -ne $hit) { return $true }
    }
    return $false
}

foreach ($cpp in $sources) {
    if (-not (Test-Path -LiteralPath $cpp)) {
        Write-Host "  [stale-check] missing source, skip: $cpp" -ForegroundColor Yellow
        continue
    }
    $obj = Join-Path $ObjDir ([IO.Path]::GetFileNameWithoutExtension($cpp) + '.obj')
    if ($Force -or -not (Test-Path -LiteralPath $obj)) {
        [void]$stale.Add($cpp)
        continue
    }

    $objTime = (Get-Item -LiteralPath $obj).LastWriteTime
    if ((Get-Item -LiteralPath $cpp).LastWriteTime -gt $objTime) {
        [void]$stale.Add($cpp)
        continue
    }

    $roots = New-Object System.Collections.Generic.List[string]
    $cppDir = Split-Path -Parent $cpp
    if ($cppDir) { [void]$roots.Add($cppDir) }
    if ($cpp -match "src[\\/]physics[\\/](ground_collision_system|chunk_body_collision)\.cpp$") {
        [void]$roots.Add("src\render")
    }

    # main.cpp includes the header-only Vulkan stack (vk_cloud_system.h, imgui_*.h, ...)
    if ([IO.Path]::GetFileName($cpp) -ieq 'main.cpp') {
        foreach ($r in @('src', 'vendor\imgui')) {
            if (Test-Path -LiteralPath $r) { [void]$roots.Add($r) }
        }
    }

    if (Test-AnyHeaderNewer -Roots $roots.ToArray() -ObjTime $objTime) {
        [void]$stale.Add($cpp)
    }
}

# One line for batch `for /f` consumption (may be empty = nothing stale)
Write-Output ($stale -join ' ')
exit 0
