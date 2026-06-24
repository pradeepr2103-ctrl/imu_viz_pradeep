# IMU Visualizer — Fixed Build

## What was wrong (and what was fixed)

### Bug 1 — GLB is in centimetre scale
The Mixamo `human.glb` stores all coordinates in centimetres (~176 units tall).
Every previous attempt used negative scales, flips, or hacks that broke winding order
→ black screen.

**Fix:** Apply a clean **uniform 0.01 scale** in `modelMatrix()` inside `Renderer.cpp`.
No rotation needed — Mixamo models are already Y-up in their local space.

```cpp
// Renderer.cpp — modelMatrix()
Mat4 m = Mat4::identity();
m.m[0] = 0.01f;  // X
m.m[5] = 0.01f;  // Y  ← positive, no flip
m.m[10]= 0.01f;  // Z
```

### Bug 2 — Camera targeted wrong Y height
`meshCenter()` returned the raw cm bbox centre (~Y=88), not the world-space centre (Y=0.88).
`buildView()` now uses a hardcoded `center = {0, 0.88, 0}` — correct for the scaled model.

### Bug 3 — FK clobbered bone translations
`computeFK` replaced the full `localBindPose` with a pure rotation matrix from the sensor
quaternion, discarding the bone's local translation. This collapsed the skeleton.

**Fix:** Copy the translation column from `localBindPose` into the rotation matrix:
```cpp
Mat4 rot = Mat4::fromQuat(pose[s]);
rot.m[12] = bind.m[12];  // preserve bone translation
rot.m[13] = bind.m[13];
rot.m[14] = bind.m[14];
local[b] = rot;
```

## Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./bharatanatyam_viz
```

## Controls
| Key | Action |
|-----|--------|
| T | Start recording teacher |
| S | Start recording student |
| R | Stop recording, save CSV |
| P | Load CSVs, play comparison |
| Space | Pause / resume playback |
| ← → | Scrub ±1 second |
| ESC | Quit |

## Sensor mapping (Skeleton.h)
| Sensor ID | Bone |
|-----------|------|
| 0 | mixamorig:Hips |
| 1 | mixamorig:Spine1 |
| 2 | mixamorig:Head |
| 3 | mixamorig:RightArm |
| 4 | mixamorig:RightForeArm |
| 5 | mixamorig:LeftArm |
| 6 | mixamorig:LeftForeArm |
| 7 | mixamorig:RightUpLeg |
| 8 | mixamorig:LeftUpLeg |
| 9 | mixamorig:RightLeg |
