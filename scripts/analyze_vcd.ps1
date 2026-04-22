# analyze_vcd.ps1 — Quick VCD analysis for turbo decoder key signals
# Looks for X-values in critical interleaved-phase signals

$vcdFile = "C:\Users\USER\Documents\Digital_VLSI_grp_10_BITS\BITS\tb_turbo_decoder.vcd"

# Read first 2000 lines to get the signal ID mapping
$headerLines = Get-Content $vcdFile -TotalCount 2000
$signalMap = @{}
$currentScope = ""

foreach ($line in $headerLines) {
    if ($line -match '^\$scope module (\S+)') {
        $currentScope = $currentScope + "." + $Matches[1]
    }
    elseif ($line -match '^\$upscope') {
        $currentScope = $currentScope -replace '\.[^.]+$', ''
    }
    elseif ($line -match '^\$var\s+\S+\s+\d+\s+(\S+)\s+(\S+)') {
        $id = $Matches[1]
        $name = $Matches[2]
        $fullName = $currentScope + "." + ($name -replace '\s.*$','')
        $signalMap[$id] = $fullName
    }
}

Write-Host "=== Signal Map (key signals) ===" -ForegroundColor Cyan

# Find signals of interest
$targetSignals = @("pi_fr_even", "pi_fr_odd", "mn_fr_e_row", "mn_fr_o_row", 
                     "mn_br_e_row", "mn_br_o_row", "mn_dbr_e_row", "mn_dbr_o_row",
                     "extr_e_rd0", "extr_o_rd0", "c0_l_extr_even", "c0_l_extr_odd",
                     "c1_l_extr_even", "c1_l_extr_odd", "is_interleaved", "half_iter_cnt",
                     "perm_fr_even", "perm_fr_odd", "main_state", "fetch_state",
                     "ld_w_en", "ld_w_data", "ld_w_data_odd")

$targetIds = @{}
foreach ($entry in $signalMap.GetEnumerator()) {
    foreach ($target in $targetSignals) {
        if ($entry.Value -like "*$target*") {
            $targetIds[$entry.Key] = $entry.Value
            Write-Host "  ID=$($entry.Key) -> $($entry.Value)"
        }
    }
}

Write-Host "`n=== Scanning for X values in target signals ===" -ForegroundColor Cyan

# Now scan the actual value changes — look for 'x' in any target signal ID
$reader = [System.IO.StreamReader]$vcdFile
$inHeader = $true
$xCount = @{}
$lineNum = 0
$currentTime = "0"
$xTimestamps = @{}  # track first X occurrence per signal

while ($null -ne ($line = $reader.ReadLine())) {
    $lineNum++
    
    if ($inHeader -and $line -eq '$enddefinitions $end') {
        $inHeader = $false
        continue
    }
    if ($inHeader) { continue }
    
    # Track time
    if ($line -match '^#(\d+)') {
        $currentTime = $Matches[1]
        continue
    }
    
    # Check for value changes with X
    foreach ($id in $targetIds.Keys) {
        if ($line -match "x.*$([regex]::Escape($id))$" -or $line -match "^x$([regex]::Escape($id))$" -or 
            ($line -like "*x*" -and $line -like "*$id")) {
            $sigName = $targetIds[$id]
            if (-not $xCount.ContainsKey($sigName)) { $xCount[$sigName] = 0 }
            $xCount[$sigName]++
            if (-not $xTimestamps.ContainsKey($sigName)) {
                $xTimestamps[$sigName] = $currentTime
            }
            # Only report first few
            if ($xCount[$sigName] -le 3) {
                Write-Host "  X at time=$currentTime signal=$sigName" -ForegroundColor Yellow
            }
        }
    }
}
$reader.Close()

Write-Host "`n=== Summary ===" -ForegroundColor Cyan
if ($xCount.Count -eq 0) {
    Write-Host "  NO X values found in any target signal!" -ForegroundColor Green
} else {
    foreach ($entry in $xCount.GetEnumerator()) {
        Write-Host "  $($entry.Value) X transitions in $($entry.Key) (first at t=$($xTimestamps[$entry.Key]))" -ForegroundColor Red
    }
}
Write-Host "Done."
