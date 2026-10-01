param([switch]$Staged)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $root
$output = Join-Path $root 'build\motor-check\firmware'
New-Item -ItemType Directory -Force -Path $output | Out-Null
$params = Get-Content -LiteralPath 'build\machinery\builder.params' -Raw | ConvertFrom-Json
$hal = 'D:/Environment/STM32CubeMX/Repository/STM32Cube_FW_F1_V1.8.7/Drivers/STM32F1xx_HAL_Driver/Src'
$sources = @($params.sourceList) + @('Motor/motor.c', '../Core/Src/tim.c', "$hal/stm32f1xx_hal_tim.c", "$hal/stm32f1xx_hal_tim_ex.c")
$sources = @($sources | ForEach-Object { $_.Replace('\','/') } | Select-Object -Unique)
$sources = @($sources | Where-Object { $_ -notmatch '(^|/)(n20|encoder)\.c$' })
$argsC = @('--cpu','Cortex-M3','--c99','--split_sections','-O2','--diag_suppress=1295')
foreach ($include in (@($params.incDirs) + @('Motor'))) { $argsC += @('-I',$include) }
foreach ($define in $params.defines) { $argsC += ('-D' + $define) }
$compiler = Join-Path $params.toolchainLocation 'bin\armcc.exe'
$assembler = Join-Path $params.toolchainLocation 'bin\armasm.exe'
$linker = Join-Path $params.toolchainLocation 'bin\armlink.exe'
$objects = @()
foreach ($source in $sources) {
    if ($Staged -and $source -eq '../Core/Src/main.c') { $source = 'build/motor-simplify/main.c' }
    $obj = Join-Path $output ([System.IO.Path]::GetFileNameWithoutExtension($source) + '.o')
    if ([System.IO.Path]::GetExtension($source) -eq '.s') {
        & $assembler --cpu Cortex-M3 $source -o $obj
    } else {
        & $compiler @argsC -c $source -o $obj
    }
    if ($LASTEXITCODE -ne 0) { throw ('Compilation failed: ' + $source) }
    $objects += $obj
}
& $linker --cpu Cortex-M3 --scatter 'build\machinery\machinery.sct' --map --list "$output\machinery.map" @objects -o "$output\machinery.axf"
if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
Write-Output ('PASS: compiled and linked ' + $objects.Count + ' source files')
