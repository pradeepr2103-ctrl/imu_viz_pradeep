# Bharatanatyam IMU Visualizer v2

Real-time dual **skinned 3D mesh** comparison for Bharatanatyam dance accuracy.
Uses `human.glb` (Mixamo rig) rendered with OpenGL 3.3 + Assimp + GLFW.

## What you see

| Mode | Window |
|------|--------|
| Live / Recording | Single skinned human mesh, warm skin tone, lit with Blinn-Phong |
| Compare (P) | Left half = Teacher (blue tint), Right half = Student (red tint), same mesh geometry |

## Controls

| Key | Action |
|-----|--------|
| T | Start recording Teacher pose |
| S | Start recording Student pose |
| R | Stop recording → saves CSV |
| P | Load CSVs → play side-by-side comparison |
| Space | Pause / resume comparison playback |
| ← → | Scrub ±1 second in comparison |
| ESC | Quit |

## Build

### 1. Install system dependencies (Ubuntu/Debian)
```bash
sudo apt install cmake build-essential \
    libglfw3-dev libassimp-dev libgl1-mesa-dev
```

### 2. Get real GLAD files  ← REQUIRED BEFORE BUILDING
1. Go to https://glad.dav1d.de
2. Set: Language=C/C++, Specification=OpenGL, gl=Version 3.3, Profile=**Core**
3. Click **GENERATE**, download the zip
4. Copy files:
   ```
   glad/include/glad/gl.h  →  third_party/glad/include/glad/gl.h
   glad/src/gl.c           →  third_party/glad/src/gl.c
   ```

### 3. Build & run
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./bharatanatyam_viz
```

The binary expects to run from the `build/` directory.
The CMake post-build step copies `visualizer/assets/` next to the binary automatically.

## UDP packet format (ESP32 → PC)

Each sensor sends 17 bytes to port 5005:

| Bytes | Content |
|-------|---------|
| 0 | Sensor ID (0–9) |
| 1–4 | qw (float, little-endian) |
| 5–8 | qx |
| 9–12 | qy |
| 13–16 | qz |

## Sensor → Bone mapping

| Sensor | Bone |
|--------|------|
| 0 | Hips |
| 1 | Spine1 |
| 2 | Head |
| 3 | RightArm |
| 4 | RightForeArm |
| 5 | LeftArm |
| 6 | LeftForeArm |
| 7 | RightUpLeg |
| 8 | LeftUpLeg |
| 9 | RightLeg |

## Output files
`recordings/teacher.csv` and `recordings/student.csv`

CSV columns: `timestamp, qw0,qx0,qy0,qz0, qw1,..., qz9`
