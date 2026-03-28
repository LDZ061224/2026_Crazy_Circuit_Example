---
name: mdk-arm-oneclick
description: "Use when: setting up Keil MDK-ARM one-click build, download, and debug for STM32 embedded projects; detect .uvprojx, auto-parse device/output settings, then generate scripts, VS Code tasks, and launch.json automatically."
---

# MDK-ARM One-Click Setup Skill

## Purpose
Standardize one-click Keil build, download, and debug setup for MDK-ARM repositories.

This skill should create (or refresh) the following files in the target repository root:
- `scripts/setup-mdk-oneclick.ps1`
- `scripts/build-keil.ps1`
- `scripts/flash-daplink.ps1`
- `.vscode/tasks.json`
- `.vscode/launch.json`
- `.gitignore`

## Inputs
- Keil project file path (`.uvprojx`), usually under `MDK-ARM/`
- Build output HEX path (usually `MDK-ARM/<project_name>/<project_name>.hex`)
- Build output AXF path (usually `MDK-ARM/<project_name>/<project_name>.axf`)
- Probe UID for CMSIS-DAP (auto-detect from `pyocd list --probes` when possible)

All above values should be auto-derived from `.uvprojx` first.

If there are multiple `.uvprojx` files, ask the user to choose one.

## Prerequisites
- Git installed (`git --version` works)
- Optional but recommended Git identity:
  - `git config --global user.name "<name>"`
  - `git config --global user.email "<email>"`
- Required tools for debug workflow:
  - `pyocd`
  - `arm-none-eabi-gdb`
  - VS Code extension `marus25.cortex-debug`

## Workflow
1. Discover project file
- Prefer: `MDK-ARM/*.uvprojx`
- Fallback: search `**/*.uvprojx`

2. Derive defaults
- Parse from `.uvprojx`:
  - `device` from `<Device>`
  - `outputDirectory` from `<OutputDirectory>`
  - `outputName` from `<OutputName>`
- Build derived paths:
  - `hexPath`: `<uvprojx_dir>/<outputDirectory>/<outputName>.hex`
  - `axfPath`: `<uvprojx_dir>/<outputDirectory>/<outputName>.axf`
- Build debug IDs:
  - `targetId = lowercase(<Device>)`
  - `probeUid = first probe UID from pyOCD, or ask user`
- Fallback when XML fields are missing:
  - `outputDirectory = <projectName>`
  - `outputName = <projectName>`
  - `device`: ask user to confirm

3. Generate setup script
- Use template `templates/setup-mdk-oneclick.ps1.template`
- Save as `scripts/setup-mdk-oneclick.ps1`

4. Generate scripts
- Use template `templates/build-keil.ps1.template`
- Use template `templates/flash-daplink.ps1.template`
- Replace tokens:
  - `__UVPROJX_REL__`
  - `__HEX_REL__`

5. Generate VS Code task file
- Use template `templates/tasks.json.template`
- Save to `.vscode/tasks.json`
- Include:
  - `pyOCD: Ensure GDB Server (50002)`
  - `Build + Ensure pyOCD`

6. Generate VS Code debug launch file
- Use template `templates/launch.json.template`
- Save to `.vscode/launch.json`
- Replace tokens:
  - `__AXF_REL__`
  - `__DEVICE__`
  - `__TARGET_ID__`

- Include 2 debug configs:
  - `DAPLink: Debug (pyOCD)` (extension-managed pyOCD)
  - `DAPLink: Debug (External pyOCD)` (recommended fallback)

7. Generate Git ignore rules
- Use template `templates/gitignore.template`
- Save to `.gitignore`
- Default behavior: ignore everything, then allowlist important source/header paths and team-shared VS Code configs.

8. Validate
- Run: `powershell -NoProfile -ExecutionPolicy Bypass -File scripts/setup-mdk-oneclick.ps1`
- Build: run task `Keil: Build (MDK)`
- Download: run task `DAPLink: Download HEX`
- Debug preferred: start launch config `DAPLink: Debug (External pyOCD)`
- Fallback internal: `DAPLink: Debug (pyOCD)`

9. Hardening requirements (must keep)
- PowerShell char trimming/replacing must use char literals, for example:
  - `.TrimStart('\\','/')`
  - `.TrimEnd('\\','/')`
  - `.Replace('\\', '/')`
- `pyOCD: Ensure GDB Server (50002)` must not fail when `__PROBE_UID__` is a placeholder.
- If UID is missing or equals `PUT_PROBE_UID_HERE`, task should auto-detect from `pyocd list --probes`.
- If auto-detect still fails, start pyOCD without `--uid` so single-probe setups still work.
- Error message on server start failure must explicitly mention `probe connection`, `UID`, and `targetId`.

10. Git bootstrap (recommended)
- If repo is not initialized, run:
  - `git init -b main`
  - `git config core.filemode false`
  - `git config core.quotepath false`
  - `git config fetch.prune true`
  - `git config pull.rebase false`
- Create `.gitignore` from template using the current team policy:
  - ignore everything by default (`*`), then allow directory traversal (`!*/`)
  - track only important source/header files:
    - `Application/**/*.c`, `Application/**/*.h`
    - `Core/Inc/**/*.h`, `Core/Src/**/*.c`
    - `Task/**/*.c`, `Task/**/*.h`
    - `Libraries/**/*.c`, `Libraries/**/*.h`
  - keep team-shared VS Code files tracked: `.vscode/tasks.json` and `.vscode/launch.json`
- First commit suggestion:
  - `git add .`
  - `git commit -m "chore: initialize mdk one-click environment"`

## Rules
- Do not hardcode machine-specific absolute paths unless user requests it.
- Keep `KEIL_UV4` environment variable override support in build script.
- Keep script defaults relative to repository root.
- Do not delete unrelated existing tasks; merge carefully if needed.
- If `cortex-debug` or `pyocd` is missing, tell user exactly which dependency is required.
- Prefer external pyOCD debug path as default stable workflow.

## Success Criteria
- User can execute `Build + DAPLink Download` task from VS Code in one action.
- Build script prints basic artifact summary and exits non-zero on failures.
- Download script detects DAPLink removable drive by label and copies HEX.
- Debug launch configurations can start from generated AXF path.
- External debug path can auto-ensure pyOCD server before attaching.
- Setup should succeed in one pass even when probe UID is not pre-filled.
- Repository can be initialized and committed without pulling in Keil temporary/user files.
