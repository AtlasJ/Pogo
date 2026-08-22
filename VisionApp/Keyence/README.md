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

## Not yet calibrated

Three constants are placeholders marked `TODO: CALIBRATE` and must be measured on the machine
before production use:

| Constant | File |
| --- | --- |
| `KEYENCE_X_PITCH_UM` | `ImageManager.cpp`, `rotate_heightMap()` |
| `KEYENCE_Y_PITCH_UM` | `ImageManager.cpp`, and `yPitchUm` in `keyence.json` — keep both in sync |
| `laser_fov_mm` | `VisionApp_CRUD.cpp` **and** `VisionApp_JSON.cpp` — duplicated, nothing syncs them |

The driver logs the real X pitch and field of view on every scan:

```
[Profiler_Keyence] ===== MEASURED LASER FOV = <mm> mm (<N> points @ <P> um) =====
```

`Y` pitch is a gantry property (encoder travel per trigger), not a sensor one, so it has to be
measured separately.

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
