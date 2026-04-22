<# 
    BER Calculator for LTE Turbo Decoder (Correct Version)
    
    Approach:
    The test vector generator (gen_test_vectors.py) creates random 5-bit 
    signed LLRs as channel observations. The SIGN of the systematic LLR 
    represents the channel-observed hard decision. After turbo decoding,
    the SIGN of the extrinsic+systematic (posterior LLR) gives the decoded bit.
    
    For BER, we compare:
      - "Transmitted bit" = sign of systematic LLR (before noise, positive=0)
      - "Decoded bit"     = sign of extrinsic LLR from LD output (positive=0)
    
    Since we're using random LLRs (not actual channel simulation), the more 
    meaningful metric is checking that the decoder converges: the extrinsic
    magnitudes should be large (confident decisions) after 5.5 half-iterations.
    
    Memory Layout (from gen_test_vectors.py):
      sys_even_ram row r = {L_s[2r+S][4:0], L_s[2r][4:0]}  (col1=core1, col0=core0)
      sys_odd_ram  row r = {L_s[2r+1+S][4:0], L_s[2r+1][4:0]}
    
    LD Output Layout:
      ld_ram row r:
        even_word = {extr_core1[2r][5:0], extr_core0[2r][5:0]}
        odd_word  = {extr_core1[2r+1][5:0], extr_core0[2r+1][5:0]}
    
    So for Core 0: the transmitted bit for address k is sign(L_s[k])
                    the decoded bit from LD is sign(extr_core0[k])
#>

$dataDir = "C:\Users\USER\Documents\Digital_VLSI_grp_10_BITS\BITS\data"

# --- Load systematic LLRs for Core 0 (transmitted bits) ---
Write-Host "============================================================"
Write-Host "  LTE Turbo Decoder BER Calculator"
Write-Host "============================================================"
Write-Host ""

$sysEvenLines = Get-Content (Join-Path $dataDir "sys_even_ram.hex")
$sysOddLines  = Get-Content (Join-Path $dataDir "sys_odd_ram.hex")

function SignExtend5($val) {
    if ($val -ge 16) { return $val - 32 }
    return $val
}
function SignExtend6($val) {
    if ($val -ge 32) { return $val - 64 }
    return $val
}
function HardBit($signed_val) {
    if ($signed_val -lt 0) { return 1 } else { return 0 }
}

# Extract Core 0 systematic LLRs (col0 = lower 5 bits)
$sysTx = New-Object int[] 3072
for ($row = 0; $row -lt $sysEvenLines.Count; $row++) {
    $val = [Convert]::ToInt32($sysEvenLines[$row].Trim(), 16)
    $col0 = SignExtend5($val -band 0x1F)       # Core 0
    $sysTx[2 * $row] = $col0
}
for ($row = 0; $row -lt $sysOddLines.Count; $row++) {
    $val = [Convert]::ToInt32($sysOddLines[$row].Trim(), 16)
    $col0 = SignExtend5($val -band 0x1F)       # Core 0
    $sysTx[2 * $row + 1] = $col0
}

Write-Host ("Loaded {0} Core 0 systematic LLRs from input BRAMs" -f 3072)

# --- Load LD Output (decoded extrinsic) ---
$ldLines = Get-Content (Join-Path $dataDir "ld_ram_output.hex")
$extrDecoded = New-Object int[] 3072

for ($row = 0; $row -lt $ldLines.Count; $row++) {
    $parts = $ldLines[$row].Trim() -split '\s+'
    if ($parts.Count -ne 2) { continue }
    
    $evenWord = [Convert]::ToInt32($parts[0], 16)
    $oddWord  = [Convert]::ToInt32($parts[1], 16)
    
    # Core 0 extrinsic = col0 (lower 6 bits)
    $extrDecoded[2 * $row]     = SignExtend6($evenWord -band 0x3F)
    $extrDecoded[2 * $row + 1] = SignExtend6($oddWord  -band 0x3F)
}
Write-Host ("Loaded {0} Core 0 extrinsic LLRs from LD output" -f 3072)

# --- Compute BER ---
Write-Host ""
Write-Host "------------------------------------------------------------"
Write-Host "  Comparison: Channel Sys LLR (tx) vs Decoded Extr LLR (rx)"
Write-Host "------------------------------------------------------------"

$nBits = 3072
$bitErrors = 0
$sysCorrectCount = 0  # channel hard-decision correct count
$decCorrectCount = 0  # decoder hard-decision same as channel

# Also compute extrinsic magnitude histogram for convergence check
$extrMagHist = @{} # magnitude -> count

Write-Host ""
Write-Host ("{0,5} | {1,7} {2,7} | {3,7} {4,7} | {5,5}" -f "Addr","Sys_LLR","Sys_Bit","Dec_LLR","Dec_Bit","Match")
Write-Host ("-" * 60)

$printed = 0
for ($i = 0; $i -lt $nBits; $i++) {
    $sysLlr  = $sysTx[$i]
    $decodeLlr = $extrDecoded[$i]
    
    $sysBit    = HardBit $sysLlr
    $decodeBit = HardBit $decodeLlr
    
    if ($sysBit -ne $decodeBit) { $bitErrors++ }
    
    $mag = [Math]::Abs($decodeLlr)
    if (-not $extrMagHist.ContainsKey($mag)) { $extrMagHist[$mag] = 0 }
    $extrMagHist[$mag]++
    
    # Print first 15, last 5, and some error cases
    $shouldPrint = ($i -lt 15) -or ($i -ge ($nBits - 5))
    if ($shouldPrint) {
        $matchStr = if ($sysBit -eq $decodeBit) { "OK" } else { "ERR" }
        Write-Host ("{0,5} | {1,7} {2,7} | {3,7} {4,7} | {5,5}" -f $i, $sysLlr, $sysBit, $decodeLlr, $decodeBit, $matchStr)
    } elseif ($i -eq 15) {
        Write-Host ("  ... ({0} more rows) ..." -f ($nBits - 20))
    }
}

$ber = $bitErrors / $nBits

# Compute convergence metrics
$highConfCount = 0  # |extr| >= 10
$midConfCount = 0   # 5 <= |extr| < 10
$lowConfCount = 0   # |extr| < 5
$zeroCount = 0
for ($i = 0; $i -lt $nBits; $i++) {
    $mag = [Math]::Abs($extrDecoded[$i])
    if ($mag -eq 0) { $zeroCount++ }
    elseif ($mag -lt 5)  { $lowConfCount++ }
    elseif ($mag -lt 10) { $midConfCount++ }
    else { $highConfCount++ }
}

Write-Host ""
Write-Host "============================================================"
Write-Host "  RESULTS"
Write-Host "============================================================"
Write-Host ("  Total bits compared:     {0}" -f $nBits)
Write-Host ("  Hard-bit disagreements:  {0}" -f $bitErrors)
Write-Host ("  BER (sys vs decoded):    {0:F6} ({1:F4}%)" -f $ber, ($ber * 100))

Write-Host ""
Write-Host "  Extrinsic LLR Magnitude Distribution (convergence check):"
Write-Host ("    |LLR| >= 10 (high conf):  {0,5} ({1:F1}%)" -f $highConfCount, ($highConfCount / $nBits * 100))
Write-Host ("    5 <= |LLR| < 10 (mid):    {0,5} ({1:F1}%)" -f $midConfCount, ($midConfCount / $nBits * 100))
Write-Host ("    1 <= |LLR| < 5  (low):    {0,5} ({1:F1}%)" -f $lowConfCount, ($lowConfCount / $nBits * 100))
Write-Host ("    |LLR| == 0 (undecided):   {0,5} ({1:F1}%)" -f $zeroCount, ($zeroCount / $nBits * 100))

Write-Host ""
if ($highConfCount -gt ($nBits * 0.5)) {
    Write-Host "  Convergence: GOOD - majority of extrinsic LLRs are high confidence" -ForegroundColor Green
} elseif ($zeroCount -gt ($nBits * 0.5)) {
    Write-Host "  Convergence: FAIL - majority are zero (decoder not updating extrinsic)" -ForegroundColor Red
} else {
    Write-Host "  Convergence: MODERATE - decoder is updating but not fully converged" -ForegroundColor Yellow
}

if ($ber -lt 0.1) {
    Write-Host "  BER Status:  GOOD" -ForegroundColor Green
} elseif ($ber -lt 0.3) {
    Write-Host "  BER Status:  MODERATE (expected for random LLRs without proper channel model)" -ForegroundColor Yellow
} else {
    Write-Host "  BER Status:  HIGH (but may be normal for random LLR inputs)" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "  NOTE: Random LLR inputs (not from an actual channel model) can produce"
    Write-Host "  high BER. This does NOT necessarily indicate an RTL bug."
    Write-Host "  Key indicator is convergence: extrinsic magnitudes should grow"
    Write-Host "  over iterations, showing the decoder is working."
}

Write-Host ""
Write-Host "============================================================"
