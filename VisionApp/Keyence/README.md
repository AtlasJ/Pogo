# Keyence LJ-X8000A SDK

Vendored SDK for `Profiler_Keyence`, the LJ-X8000A backend behind the `"KeyenceLJ"` API string
in `C:/Advanced/Data/config/profiler.json`.

## Two files are NOT in git

The repo's `.gitignore` excludes `*.lib`, `*.dll` and `x64/`, so a fresh clone will **fail to
link** until you copy two binaries by hand. This matches how SSZN and SmartRay already work in
this project — only their headers are tracked too.

| File | Tracked? | Where it has to go |
| --- | --- | --- |
| `include/LJX8_IF.h` | yes | already here |
| `include/LJX8_ErrorCode.h` | yes | already here |
| `lib/LJX8_IF.lib` | **no** | `VisionApp\Keyence\lib\` |
| `LJX8_IF.dll` | **no** | `x64\Release\` (beside `VisionApp.exe`) |

## Where to get them

Both ship inside **LJ-X Navigator**, not as a separate SDK download:

```
C:\Program Files\KEYENCE\LJ-X Navigator\lib\Sample_ImageAcquisition\src\lib\x64\
    LJX8_IF.lib
    LJX8_IF.dll
```

Download LJ-X Navigator from Keyence's support site for the LJ-X8000A. You do **not** need it
installed to build — the two files are self-contained and can simply be shared.

> **Take the `x64\` copies.** The SDK ships x86 and x64 side by side with identical filenames.
> Picking the wrong one gives `module machine type 'x86' conflicts with target machine type
> 'x64'` at link, which is confusing because the filename looks correct.

There is no Debug variant of the lib — one `LJX8_IF.lib` serves both configurations. If you build
`Debug|x64` you also need `LJX8_IF.dll` in `x64\Debug\`, since that is where `OutDir` points.

## Config templates

`config/` holds annotated examples, not live config. Machine config lives outside the repo:

- `config/profiler.keyence.example.json` → copy into `C:/Advanced/Data/config/profiler.json`
- `config/keyence.json` → copy into `C:/Advanced/Data/config/keyence.json`

Back up whatever is already there first; that folder's convention is to keep dated variants
alongside rather than edit in place.

The API string must be `"KeyenceLJ"`, and the per-recipe `"laserApi"` in `recipeConfig.json` must
match it. It is spelled that way rather than `"Keyence"` because `SRXManager` already integrates
a Keyence SR-X100 barcode reader — the bare brand no longer identifies a device in this tree.

## Hardware on this machine

**LJ-X8060 head, LJ-X8000A controller** (confirmed 2026-08-24). The sensor lives on the machine's
IPC, not on a development laptop.

Data sheet figures the code depends on:

| Spec | Value |
| --- | --- |
| Reference distance | 64 mm |
| Z measurement range | ±7.3 mm (F.S. 14.6 mm) |
| X measurement range | 15 mm NEAR → 16 mm FAR |
| Profile data interval | **5 µm** default, adjustable from 4 µm up |
| Profile data count | **3200 points** |
| Repeatability | 0.4 µm Z, 0.5 µm X |
| Linearity | ±0.04% of F.S. (≈ ±5.8 µm) |

`5 µm × 3200 = 16 mm`, which is why `KEYENCE_X_PITCH_UM` is 5.0 and `laser_fov_mm` is 16.0.
**Changing the profile data interval in Navigator moves both**, and nothing in the code detects it.

The head model matters in one place automatically: `zPitchForHead()` maps `8060` to **0.8 µm per
grey level**, so the Z axis needs no configuration. Verify it on first connect —

```
[Profiler_Keyence] Head model  : LJ-X8060
[Profiler_Keyence] Z pitch     : 0.80 um per grey level
```

If that second line reads `1.60`, the reported model string did not match the `contains("8060")`
test and every height will be out by exactly 2×.

## Calibration status

| Constant | File | Status |
| --- | --- | --- |
| `KEYENCE_X_PITCH_UM` = 5.0 | `ImageManager.cpp`, `rotate_heightMap()` | **From the data sheet.** Confirmed by the controller: `MEASURED LASER FOV = 16.0000 mm (3200 points @ 5.0000 um)` |
| `laser_fov_mm` = 16.0 | `VisionApp_CRUD.cpp` **and** `VisionApp_JSON.cpp` — duplicated, nothing syncs them | **From the data sheet** (5 µm × 3200) |
| `yPitchUm` | `keyence.json` **only** — no longer a code constant | **Measured 2026-08-27: 4.0 µm per encoder count** |

`yPitchUm` used to be duplicated as a `KEYENCE_Y_PITCH_UM` constant in `ImageManager.cpp` that had to
be kept equal to `keyence.json` by hand, with nothing detecting a mismatch. That constant is gone:
`rotate_heightMap` now reads `ProfilerManager::getLinePitchUm()`, which returns the driver's
`yPitchUm × divider`. One source of truth, and the divider is the one `setDivider()` actually pushed
rather than whichever optics entry happened to be first in the hash.

It is editable on the **3D Optics page → Profiler Hardware**, guarded by a confirmation and an
`YPITCH_UPDATE` audit entry, because it is a property of the machine's gantry rather than of the head
or the code — so it should never have needed a rebuild.

**Measured on Codetrace-CK:** 155.999 mm of travel against 39000 encoder counts = **4.0000 µm/count**
at 2-phase ×1. The Profiler Scan Test reports this as `Encoder scaling` on every run.

One known simplification in the X scale: the sheet gives 15 mm at the NEAR limit and 16 mm at FAR,
so the true pitch varies about 2% across the ±7.3 mm Z range. A single constant cannot express
that, so X is exact at FAR and reads ~2% narrow at NEAR. Below the other error sources for now.

The driver logs the real X pitch and field of view on every scan:

```
[Profiler_Keyence] ===== MEASURED LASER FOV = <mm> mm (<N> points @ <P> um) =====
```

`Y` pitch is a gantry property (encoder travel per trigger), not a sensor one, so it has to be
measured separately.

## The two TCP ports

The controller listens on **two** ports and the manual (p.69) forbids making them equal:

> "Do not set the command port number and the high-speed port number to the same number."

| Key in `keyence.json` | Default | Maps to |
| --- | --- | --- |
| `commandPort` | 24691 | `LJX8IF_ETHERNET_CONFIG.wPortNo`, controller setting item 07h |
| `highSpeedPort` | 24692 | `wHighSpeedPortNo`, controller setting item 08h |

If you change either in Navigator, change it here to match.

**Failure signature when they are equal** — and this one is misleading, because it looks like a
network fault and is not:

```
[Profiler_Keyence] InitializeHighSpeedSimpleArray failed rc=0x1000
[Profiler] Failed to connect profiler: profiler1
```

`0x1000` is `LJX8IF_RC_ERR_OPEN`. Note there is **no** `EthernetOpen failed` line — the command
channel opens fine, because the controller really is listening on that port. The high-speed open
then collides with our own command socket. Both a `loadConfig` check and an `initHighSpeed` guard
now reject an equal pair before it gets that far.

## Reference

The setting codes used by every `setSetting()` call in `Profiler_Keyence.cpp` come from section
11.3 of the communication library manual, and the batch acquisition sequence from section 12.3:

```
C:\Program Files\KEYENCE\LJ-X Navigator\lib\Manual\en\
    LJ-X8000A_CommLib_RM_N06GB_122330_GB_2105-1.pdf
```

Keyence's own `Sample_ImageAcquisition` is worth reading for the call sequence, but its
`LJXA_ACQ` wrapper is not used here — it busy-waits in a polling loop and mallocs per
acquisition, neither of which is acceptable inside `JobThread::scan()`.
