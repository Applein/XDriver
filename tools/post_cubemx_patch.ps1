<#
    post_cubemx_patch.ps1

    Re-apply the RT-Thread + FinSH integration to the STM32CubeMX generated
    files. CubeMX overwrites everything outside the USER CODE sections, so the
    following edits must be restored every time you regenerate code:

      1. startup_*.s          Reset_Handler:  bl main  ->  bl entry
      2. *.ld (linker script) add .rti_fn / .fsymtab / .vsymtab output sections
      3. Core/Src/stm32f1xx_it.c
                              remove HardFault_Handler and PendSV_Handler
                              (provided by RT-Thread libcpu context_gcc.S)
                              SysTick_Handler -> rt_tick_increase()
                              USART1_IRQHandler -> rt_hw_console_isr()
      4. Core/Src/main.c      MX_*_Init(): drop "static" (rt_board.c calls them)
                              remove HAL_Init / SystemClock_Config / MX_*_Init
                              from main() (already done in rt_hw_board_init)
      5. CMakeLists.txt       include(cmake/rtthread.cmake)

    This script is idempotent: running it twice changes nothing.

    Usage:
        powershell -ExecutionPolicy Bypass -File tools\post_cubemx_patch.ps1
        powershell -ExecutionPolicy Bypass -File tools\post_cubemx_patch.ps1 -Check

    -Check only reports what would change; no file is written.
#>

[CmdletBinding()]
param(
    [switch]$Check
)

$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
Set-Location -Path $root

$script:changeCount = 0
$script:warnCount = 0

function Get-NewLine([string]$text) {
    if ($text.Contains("`r`n")) { return "`r`n" }
    return "`n"
}

function Read-Text([string]$path) {
    return [System.IO.File]::ReadAllText($path)
}

function Save-Text([string]$path, [string]$text) {
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($path, $text, $utf8NoBom)
}

function Add-Warning([string]$msg) {
    $script:warnCount++
    Write-Host "  [WARN]  $msg" -ForegroundColor Yellow
}

function Add-Skip([string]$msg) {
    Write-Host "  [OK]    $msg" -ForegroundColor DarkGray
}

function Apply-Patch([string]$path, [string]$old, [string]$new, [string]$desc) {
    if ($old -ceq $new) {
        Add-Skip "$desc (already applied)"
        return
    }
    $script:changeCount++
    if ($Check) {
        Write-Host "  [TODO]  $desc" -ForegroundColor Cyan
        return
    }
    Save-Text $path $new
    Write-Host "  [FIXED] $desc" -ForegroundColor Green
}

# ---------------------------------------------------------------------------
# helpers: locate a C function body with brace matching
# ---------------------------------------------------------------------------
function Find-Block([string]$text, [string]$signature) {
    $i = $text.IndexOf($signature, [System.StringComparison]::Ordinal)
    if ($i -lt 0) { return $null }

    $b = $text.IndexOf('{', $i)
    if ($b -lt 0) { return $null }

    $depth = 0
    for ($p = $b; $p -lt $text.Length; $p++) {
        $ch = $text[$p]
        if ($ch -eq '{') {
            $depth++
        }
        elseif ($ch -eq '}') {
            $depth--
            if ($depth -eq 0) {
                return [pscustomobject]@{
                    SignatureIndex = $i
                    BraceStart     = $b
                    BraceEnd       = $p
                }
            }
        }
    }
    return $null
}

# Remove a whole function including its leading /** ... */ documentation block.
function Remove-Function([string]$text, [string]$signature, [string]$replacement) {
    $blk = Find-Block $text $signature
    if (-not $blk) { return $null }

    $start = $blk.SignatureIndex

    $docOpen = $text.LastIndexOf('/**', $start, [System.StringComparison]::Ordinal)
    if ($docOpen -ge 0) {
        $docClose = $text.LastIndexOf('*/', $start, [System.StringComparison]::Ordinal)
        if ($docClose -gt $docOpen) {
            $between = $text.Substring($docClose + 2, $start - ($docClose + 2))
            if ($between.Trim().Length -eq 0) { $start = $docOpen }
        }
    }

    $end = $blk.BraceEnd + 1
    if ($end -lt $text.Length -and $text[$end] -eq "`r") { $end++ }
    if ($end -lt $text.Length -and $text[$end] -eq "`n") { $end++ }

    $nl = Get-NewLine $text
    $rep = $replacement.Replace("`n", $nl)
    return $text.Substring(0, $start) + $rep + $text.Substring($end)
}

# ---------------------------------------------------------------------------
# prerequisites
# ---------------------------------------------------------------------------
Write-Host ""
Write-Host "XDriver post-CubeMX patch" -ForegroundColor White
Write-Host "  root: $root"
if ($Check) { Write-Host "  mode: CHECK ONLY (no files written)" -ForegroundColor Cyan }
Write-Host ""

$prereq = @(
    'rtconfig.h',
    'cmake/rtthread.cmake',
    'Core/Inc/rt_board.h',
    'Core/Src/rt_board.c',
    'CMakeLists.txt'
)
foreach ($p in $prereq) {
    if (-not (Test-Path (Join-Path $root $p))) {
        Add-Warning "missing integration file: $p (restore it before building)"
    }
}

# ---------------------------------------------------------------------------
# 1. startup file: Reset_Handler must branch to entry() instead of main()
# ---------------------------------------------------------------------------
Write-Host "[1/5] startup file" -ForegroundColor White
$startup = Get-ChildItem -Path $root -Filter 'startup_*.s' -File | Select-Object -First 1
if ($null -eq $startup) {
    Add-Warning "startup_*.s not found in project root"
}
else {
    $t = Read-Text $startup.FullName
    if ($t -match '(?m)^\s*bl\s+entry\b') {
        Add-Skip "$($startup.Name): Reset_Handler already calls entry()"
    }
    else {
        $new = [regex]::Replace($t, '(?m)^([ \t]*)bl[ \t]+main\b', '$1bl entry')
        if ($new -ceq $t) {
            Add-Warning "$($startup.Name): 'bl main' not found (unexpected startup file?)"
        }
        else {
            Apply-Patch $startup.FullName $t $new "$($startup.Name): bl main -> bl entry"
        }
    }
}

# ---------------------------------------------------------------------------
# 2. linker script: keep RT-Thread init table and FinSH command tables
# ---------------------------------------------------------------------------
Write-Host "[2/5] linker script" -ForegroundColor White
$linker = Get-ChildItem -Path $root -Filter '*.ld' -File | Select-Object -First 1
if ($null -eq $linker) {
    Add-Warning "*.ld not found in project root"
}
else {
    $t = Read-Text $linker.FullName
    if ($t -match '\.rti_fn\s*:') {
        Add-Skip "$($linker.Name): .rti_fn / .fsymtab / .vsymtab already present"
    }
    else {
        $nl = Get-NewLine $t
        $sections = @'
  /* RT-Thread: component auto-initialization table (INIT_BOARD_EXPORT, ...) */
  .rti_fn :
  {
    . = ALIGN(4);
    __rt_init_start = .;
    KEEP(*(SORT(.rti_fn.*)))
    __rt_init_end = .;
    . = ALIGN(4);
  } >FLASH

  /* RT-Thread FinSH: msh command table (MSH_CMD_EXPORT) */
  .fsymtab :
  {
    . = ALIGN(4);
    __fsymtab_start = .;
    KEEP(*(FSymTab))
    __fsymtab_end = .;
    . = ALIGN(4);
  } >FLASH

  /* RT-Thread FinSH: variable symbol table (FINSH_FUNCTION_EXPORT) */
  .vsymtab :
  {
    . = ALIGN(4);
    __vsymtab_start = .;
    KEEP(*(VSymTab))
    __vsymtab_end = .;
    . = ALIGN(4);
  } >FLASH

'@
        $sections = $sections.Replace("`r`n", "`n").Replace("`n", $nl)

        $anchor = '/* Constant data goes into FLASH */'
        $idx = $t.IndexOf($anchor, [System.StringComparison]::Ordinal)
        if ($idx -lt 0) {
            Add-Warning "$($linker.Name): anchor '$anchor' not found, add the sections manually"
        }
        else {
            # keep the indentation of the anchor line
            $lineStart = $t.LastIndexOf("`n", $idx) + 1
            $indent = $t.Substring($lineStart, $idx - $lineStart)
            $new = $t.Substring(0, $lineStart) + $sections + $indent + $t.Substring($idx)
            Apply-Patch $linker.FullName $t $new "$($linker.Name): add .rti_fn / .fsymtab / .vsymtab"
        }
    }
}

# ---------------------------------------------------------------------------
# 3. interrupt vectors
# ---------------------------------------------------------------------------
Write-Host "[3/5] Core/Src/stm32f1xx_it.c" -ForegroundColor White
$itc = Join-Path $root 'Core/Src/stm32f1xx_it.c'
if (-not (Test-Path $itc)) {
    Add-Warning "Core/Src/stm32f1xx_it.c not found"
}
else {
    $t = Read-Text $itc
    $nl = Get-NewLine $t

    # 3.0 includes
    if ($t -notmatch '#include\s+<rtthread\.h>') {
        $ins = "$nl#include <rtthread.h>$nl#include `"rt_board.h`""
        $new = $t -replace '(?m)^([ \t]*)(/\* USER CODE BEGIN Includes \*/)[ \t\r]*$', ('$1$2' + $ins)
        if ($new -ceq $t) {
            Add-Warning "stm32f1xx_it.c: 'USER CODE BEGIN Includes' marker missing, add includes manually"
        }
        else {
            Apply-Patch $itc $t $new "stm32f1xx_it.c: add rtthread.h / rt_board.h includes"
            $t = $new
        }
    }
    else {
        Add-Skip "stm32f1xx_it.c: includes OK"
        if ($t -notmatch '#include\s+"rt_board\.h"') {
            $new = $t -replace '(?m)^(#include\s+<rtthread\.h>[ \t]*)$', ('$1' + $nl + '#include "rt_board.h"')
            Apply-Patch $itc $t $new "stm32f1xx_it.c: add rt_board.h include"
            $t = $new
        }
    }

    # 3.1 HardFault_Handler is provided by RT-Thread (libcpu/arm/cortex-m3/context_gcc.S)
    if ($t -match 'void\s+HardFault_Handler\s*\(\s*void\s*\)') {
        $replacement = @'
/* HardFault_Handler is provided by RT-Thread
 * (RT-Thread/libcpu/arm/cortex-m3/context_gcc.S) and prints the fault
 * registers through the console. Defining it here causes a duplicate symbol. */
'@
        $new = Remove-Function $t 'void HardFault_Handler(void)' $replacement
        Apply-Patch $itc $t $new "stm32f1xx_it.c: remove HardFault_Handler (RT-Thread provides it)"
        $t = $new
    }
    else {
        Add-Skip "stm32f1xx_it.c: HardFault_Handler already removed"
    }

    # 3.2 PendSV_Handler performs the context switch
    if ($t -match 'void\s+PendSV_Handler\s*\(\s*void\s*\)') {
        $replacement = @'
/* PendSV_Handler is provided by RT-Thread
 * (RT-Thread/libcpu/arm/cortex-m3/context_gcc.S) and performs the thread
 * context switch. Defining it here would break the scheduler. */
'@
        $new = Remove-Function $t 'void PendSV_Handler(void)' $replacement
        Apply-Patch $itc $t $new "stm32f1xx_it.c: remove PendSV_Handler (RT-Thread provides it)"
        $t = $new
    }
    else {
        Add-Skip "stm32f1xx_it.c: PendSV_Handler already removed"
    }

    # 3.3 SysTick drives the RT-Thread OS tick (HAL time base stays on TIM1)
    if ($t -match 'rt_tick_increase') {
        Add-Skip "stm32f1xx_it.c: SysTick already feeds rt_tick_increase()"
    }
    else {
        $blk = Find-Block $t 'void SysTick_Handler(void)'
        if ($null -eq $blk) {
            Add-Warning "stm32f1xx_it.c: SysTick_Handler not found"
        }
        else {
            $body = $t.Substring($blk.BraceStart, $blk.BraceEnd - $blk.BraceStart + 1)
            $ins = "$nl  rt_interrupt_enter();$nl  rt_tick_increase();$nl  rt_interrupt_leave();"
            $newBody = $body -replace '(?m)^([ \t]*)(/\* USER CODE BEGIN SysTick_IRQn 0 \*/)[ \t\r]*$', ('$1$2' + $ins)
            if ($newBody -ceq $body) {
                $newBody = $body.Insert(1, $ins)
            }
            $new = $t.Substring(0, $blk.BraceStart) + $newBody + $t.Substring($blk.BraceEnd + 1)
            Apply-Patch $itc $t $new "stm32f1xx_it.c: SysTick_Handler -> rt_tick_increase()"
            $t = $new
        }
    }

    # 3.4 USART1 interrupt feeds the FinSH console ring buffer
    if ($t -match 'rt_hw_console_isr') {
        Add-Skip "stm32f1xx_it.c: USART1_IRQHandler already feeds rt_hw_console_isr()"
    }
    else {
        $blk = Find-Block $t 'void USART1_IRQHandler(void)'
        if ($null -eq $blk) {
            Add-Warning "stm32f1xx_it.c: USART1_IRQHandler not found (console disabled in CubeMX?)"
        }
        else {
            $body = $t.Substring($blk.BraceStart, $blk.BraceEnd - $blk.BraceStart + 1)
            $body = $body -replace '(?m)^[ \t]*HAL_UART_IRQHandler\(&huart1\);[ \t]*\r?\n', ''
            $ins = "$nl  /* RT-Thread console: direct register ISR, see rt_board.c */$nl  rt_interrupt_enter();$nl  rt_hw_console_isr();$nl  rt_interrupt_leave();"
            $newBody = $body -replace '(?m)^([ \t]*)(/\* USER CODE BEGIN USART1_IRQn 0 \*/)[ \t\r]*$', ('$1$2' + $ins)
            if ($newBody -ceq $body) {
                $newBody = $body.Insert(1, $ins)
            }
            $new = $t.Substring(0, $blk.BraceStart) + $newBody + $t.Substring($blk.BraceEnd + 1)
            Apply-Patch $itc $t $new "stm32f1xx_it.c: USART1_IRQHandler -> rt_hw_console_isr()"
            $t = $new
        }
    }
}

# ---------------------------------------------------------------------------
# 4. main.c
# ---------------------------------------------------------------------------
Write-Host "[4/5] Core/Src/main.c" -ForegroundColor White
$mainc = Join-Path $root 'Core/Src/main.c'
if (-not (Test-Path $mainc)) {
    Add-Warning "Core/Src/main.c not found"
}
else {
    $t = Read-Text $mainc
    $nl = Get-NewLine $t

    # 4.0 includes (USER CODE section survives regeneration, but be safe)
    if ($t -notmatch '#include\s+<rtthread\.h>') {
        $ins = "$nl#include <rtthread.h>$nl#include `"rt_board.h`""
        $new = $t -replace '(?m)^([ \t]*)(/\* USER CODE BEGIN Includes \*/)[ \t\r]*$', ('$1$2' + $ins)
        if ($new -ceq $t) {
            Add-Warning "main.c: 'USER CODE BEGIN Includes' marker missing"
        }
        else {
            Apply-Patch $mainc $t $new "main.c: add rtthread.h / rt_board.h includes"
            $t = $new
        }
    }
    else {
        Add-Skip "main.c: includes OK"
    }

    # 4.1 MX_*_Init must be callable from rt_board.c -> remove "static"
    $pattern = '(?m)^[ \t]*static[ \t]+void[ \t]+(MX_\w+_Init)[ \t]*\([ \t]*void[ \t]*\)'
    if ($t -match $pattern) {
        $new = [regex]::Replace($t, $pattern, 'void $1(void)')
        Apply-Patch $mainc $t $new "main.c: remove 'static' from MX_*_Init (called by rt_board.c)"
        $t = $new
    }
    else {
        Add-Skip "main.c: MX_*_Init already non-static"
    }

    # 4.2 main() must not initialise HAL/clock/peripherals again
    $lines = $t -split $nl
    $mainStart = -1
    $userCode2 = -1
    for ($i = 0; $i -lt $lines.Count; $i++) {
        if ($mainStart -lt 0 -and $lines[$i] -match '^\s*int\s+main\s*\(\s*void\s*\)') {
            $mainStart = $i
        }
        elseif ($mainStart -ge 0 -and $lines[$i] -match '/\*\s*USER CODE BEGIN 2\s*\*/') {
            $userCode2 = $i
            break
        }
    }
    if ($mainStart -lt 0 -or $userCode2 -lt 0) {
        Add-Warning "main.c: main() body markers not found, check it manually"
    }
    else {
        $drop = '^\s*(HAL_Init|SystemClock_Config|MX_\w+_Init)\s*\(\s*\)\s*;\s*$'
        $removed = 0
        $kept = New-Object System.Collections.ArrayList
        for ($i = 0; $i -lt $lines.Count; $i++) {
            if ($i -gt $mainStart -and $i -lt $userCode2 -and $lines[$i] -match $drop) {
                $removed++
                continue
            }
            [void]$kept.Add($lines[$i])
        }
        if ($removed -eq 0) {
            Add-Skip "main.c: main() already free of HAL/clock init calls"
        }
        else {
            $new = $kept -join $nl
            Apply-Patch $mainc $t $new "main.c: remove HAL/clock/peripheral init from main() ($removed call(s))"
            $t = $new
        }
    }
}

# ---------------------------------------------------------------------------
# 5. CMakeLists.txt: pull in the RT-Thread sources
# ---------------------------------------------------------------------------
Write-Host "[5/5] CMakeLists.txt" -ForegroundColor White
$cml = Join-Path $root 'CMakeLists.txt'
if (-not (Test-Path $cml)) {
    Add-Warning "CMakeLists.txt not found"
}
else {
    $t = Read-Text $cml
    if ($t -match 'include\s*\(\s*cmake/rtthread\.cmake\s*\)') {
        Add-Skip "CMakeLists.txt: rtthread.cmake already included"
    }
    else {
        $nl = Get-NewLine $t
        $anchor = 'add_subdirectory(cmake/stm32cubemx)'
        $idx = $t.IndexOf($anchor, [System.StringComparison]::Ordinal)
        if ($idx -lt 0) {
            Add-Warning "CMakeLists.txt: '$anchor' not found, add include(cmake/rtthread.cmake) manually"
        }
        else {
            $ins = "$nl$nl# Add RT-Thread kernel + FinSH + board port$nlinclude(cmake/rtthread.cmake)"
            $pos = $idx + $anchor.Length
            $new = $t.Substring(0, $pos) + $ins + $t.Substring($pos)
            Apply-Patch $cml $t $new "CMakeLists.txt: include(cmake/rtthread.cmake)"
        }
    }
}

# ---------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------
Write-Host ""
if ($script:warnCount -eq 0 -and $script:changeCount -eq 0) {
    Write-Host "Nothing to do - integration is intact." -ForegroundColor Green
}
elseif ($Check) {
    Write-Host "$($script:changeCount) patch(es) needed, $($script:warnCount) warning(s)." -ForegroundColor Cyan
    Write-Host "Run again without -Check to apply them."
}
else {
    Write-Host "$($script:changeCount) patch(es) applied, $($script:warnCount) warning(s)." -ForegroundColor Green
    Write-Host "Next: cmake --preset Debug ; cmake --build --preset Debug"
}
Write-Host ""
