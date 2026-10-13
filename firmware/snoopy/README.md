# SNOOPY firmware extracted from PR #2

This directory exposes the source from the ZIP committed in PR #2, without changing firmware behavior. The existing `config.mcc/` project is separate and has not been overwritten.

## Provenance

- Repository: NeilPraj/TeamFishy
- PR: https://github.com/NeilPraj/TeamFishy/pull/2
- Archive commit: 9429b757cddf2d79027b4cb5819e98aa6f54becb
- Archive: `SNOOPY 0.22 Line Follower code with serial comm and basic functions 260915 (2).zip`
- Archive SHA-256: `c8b55239ad61f71236ff1ee89c032c07b5a1f2a71f5b9f8be85152b7e4aea918`

Extracted files are byte-for-byte identical to their archive entries. Included: application C code, MCC C/header files and manifests, SNOOPY.mc3, VS Code/MPLAB project settings, CMake entry points, and shared legacy MPLAB project files. Excluded: build/, _build/, dist/, out/, generated CMake caches, nbproject/private/, local makefile settings, and MCC backup files. The original ZIP is retained as provenance, not as the review surface.

## Review entry points

- `main.c`: application logic, sensor interpretation, PD control, encoders, MPU-6500 and recovery.
- `mcc_generated_files/`: peripheral implementation and configuration.
- `.vscode/SNOOPY.mplab.json`: VS Code project targeting PIC18F47Q43.
- `SNOOPY.mc3`: MCC configuration.

## Build status and procedure

No fresh compile was performed during extraction: Microchip XC8 is not installed in the extraction environment. Old binaries in the archive do not prove that this exact source compiles.

Open this directory as the workspace in VS Code with Microchip MPLAB extensions. Install/configure XC8 and the required device pack, regenerate local CMake build files using MPLAB, then perform a clean build. The archived settings reference XC8 v3.10 via a Windows-local path; select the installed compiler rather than relying on that path. CMake presets require regenerated `.generated/` files and are not standalone build instructions.

The archived `SNOOPY.mc3` and legacy `nbproject/configurations.xml` target PIC18F26Q43, while `main.c` and the VS Code project target PIC18F47Q43. The board owner confirmed PIC18F47Q43 as the actual device. The archived F26 configuration is stale. Select PIC18F47Q43 in MCC and the legacy project, regenerate the drivers as needed, and review those changes before building. These settings are preserved for review, not silently corrected during extraction.

Before merging, review the visible source and record a clean build with compiler version, device pack version, device target, exact commit and build log. Before running on the robot, verify the physical assumptions listed at the top of main.c and test peripherals and motor behavior safely.

PR #2 should remain unmerged in its ZIP-only form. Review this source branch against PR #2's base branch (`ChatGPT_Base_Turnkey`), then use the reviewed source branch as the replacement or supplement; merely creating this branch does not change PR #2's Files changed view.
