# AGENTS.md

## Platform

- **Chip**: Infineon AURIX TC264D (TC26xB B-Step, dual-core TriCore)
- **Toolchain**: TASKING Tricore C/C++ Compiler (`ctc`), only available inside AURIX Development Studio (ADS, Eclipse CDT)
- **Cannot build or run on a normal PC** — there is no simulator. Builds target the physical MCU; output is `Debug/2026_Crazy_Circuit.elf` + `.hex`.
- **CLI build** (requires TASKING in PATH): `make -C Debug -j`
- **Flash/debug**: winIDEA via USB DAS → load `.elf` → burn to Flash

## File layout

```
user/          ← CPU entry + ISRs (cpu0_main.c, cpu1_main.c, isr.c, isr_config.h)
code/          ← Application: Ctrl.c/h (control core), Fun.c/h (peripheral init),
                  pid.c/h (PID lib), Racing_Track.c (track maps), OLED/ (display+keyboard)
libraries/     ← Third-party (Infineon iLLD + SEEKFREE v3.4.1). Do NOT edit.
Debug/         ← Build artifacts (.o, .elf, .hex)
```

## Critical include convention

**Every `.c` file includes ONLY `headfiles.h`** (no individual module headers).

`headfiles.h` (in `code/`) chains: `zf_common_headfile.h` → `pid.h` → `Fun.h` → `TCA9555.h` → `OLEDKeyboard.h` → `Ctrl.h`

When adding a module, edit `headfiles.h` — do not add per-file includes.

## Interrupt configuration (isr_config.h)

All interrupt priorities must be **unique** (range 1–255; higher = higher priority — opposite of ARM Cortex-M). Duplicate priorities → hardware exception.

DMA-triggered interrupts have a restricted range (0–47).

## Pin gotchas

- **P14.2–P14.6, P10.5, P10.6**: BOOT pins. Using them as peripherals can brick the chip.
- **P20.2**: input only, cannot be configured as output.
- **P21.6**: unavailable on TC264DA (ok on TC264D).

## Linker / dual-core

`Lcf_Tasking_Tricore_Tc.lsl` sets `LCF_DEFAULT_HOST = LCF_CPU1`, so **global variables default to CPU1 DSPRAM**. Code running on CPU0 that accesses globals must account for this (depends on linker script placement).

## Architecture summary

- `cpu0_main.c` initializes hardware, then runs OLED menu + `Vofa_Send_Data()` in a loop
- `cpu1_main.c` refreshes OLED display
- Core control runs in **3ms PIT interrupt** on CPU0 → `Car_Go()` in `Ctrl.c`
- Two operating modes: **Build_Mode** (photoelectric tracing, records mileage to Flash) and **Remember_Mode** (loads Flash data, replays with dual edge+mileage triggering)
- `Run_Mode` state machine inside `Car_Go`: Normal → Turn_Left/Right → Mileage_Mode → Straight_Mode

## Reference

`CLAUDE.md` contains detailed architecture docs, state machine diagrams, PID cascade, Flash layout, and the full Check-Edge routing table. Read it when you need deeper context than this file provides.

`Readme.md` is a static analysis report with bug fixes, known limitations, and modification history.
