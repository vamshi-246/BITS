$dir = "c:\Users\USER\Documents\Digital_VLSI_grp_10_BITS\BITS\data"
$files = Get-ChildItem -Path $dir -Filter "*.hex"

foreach ($file in $files) {
    if ($file.Name -eq "input_llr.hex") { continue }
    
    $newName = $file.Name.Replace(".hex", "_readable.txt")
    $out_path = Join-Path -Path $dir -ChildPath $newName
    
    Write-Host "Processing $($file.Name) -> $newName"
    
    $lines = Get-Content $file.FullName
    $outText = New-Object System.Collections.Generic.List[string]
    $outText.Add("ADDR | HEX  |       BINARY       | UNSIGNED |   S5 |   S6 |   S7 |  S10 |  S12 |  S16")
    $outText.Add("---------------------------------------------------------------------------------------")
    
    $implicit_addr = 0
    foreach ($line in $lines) {
        $trim = $line.Trim()
        if ($trim.Length -gt 0) {
            $addr = $implicit_addr
            $dataStr = $trim
            
            if ($trim -match "^([0-9a-fA-F]+)\s*:\s*(.+)$") {
                $addr = [Convert]::ToInt32($matches[1], 16)
                $dataStr = $matches[2] -replace "\s+",""
            }
            
            try {
                $val = [Convert]::ToInt32($dataStr, 16)
                $bin = [Convert]::ToString($val, 2).PadLeft(16, '0')
                if ($bin.Length -gt 16) { $bin = $bin.Substring($bin.Length - 16) }
                
                $bin_fmt = $bin.Substring(0,4) + "_" + $bin.Substring(4,4) + "_" + $bin.Substring(8,4) + "_" + $bin.Substring(12,4)
                
                $s5 = $val % 32
                if (($s5 -band 0x10) -ne 0) { $s5 = $s5 - 32 }
                
                $s6 = $val % 64
                if (($s6 -band 0x20) -ne 0) { $s6 = $s6 - 64 }
                
                $s7 = $val % 128
                if (($s7 -band 0x40) -ne 0) { $s7 = $s7 - 128 }
                
                $s10 = $val % 1024
                if (($s10 -band 0x200) -ne 0) { $s10 = $s10 - 1024 }
                
                $s12 = $val % 4096
                if (($s12 -band 0x800) -ne 0) { $s12 = $s12 - 4096 }
                
                $s16 = $val % 65536
                if (($s16 -band 0x8000) -ne 0) { $s16 = $val - 65536 }
                
                $outStr = "{0,-4} | {1,-4} | {2,18} | {3,8} | {4,4} | {5,4} | {6,4} | {7,4} | {8,4} | {9,4}" -f $addr.ToString("X3"), $dataStr, $bin_fmt, $val, $s5, $s6, $s7, $s10, $s12, $s16
                $outText.Add($outStr)
            } catch {
                $err = "{0,-4} | ERROR parsing {1}" -f $addr.ToString("X3"), $trim
                $outText.Add($err)
            }
            $implicit_addr++
        }
    }
    
    [System.IO.File]::WriteAllLines($out_path, $outText)
}
Write-Host "All hex files converted successfully!"
