Place this in the project root.
markdown

# ESP32 Body Tracking System – Bharatanatyam Motion Capture & Comparison

A complete, professional system for **real‑time full‑body motion capture** using
10 ESP32‑C3 nodes with MPU‑6050 sensors, streaming quaternions over WiFi/UDP to
an OpenGL‑based visualizer on Ubuntu. It records **teacher and student
performances** into CSV files and plays them back **side‑by‑side** with
differently‑coloured skeletons, enabling direct visual comparison of
Bharatanatyam movements.

---

## Table of Contents
- [System Architecture](#system-architecture)
- [Folder Structure](#folder-structure)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies (PC)](#software-dependencies-pc)
- [Setup & Build](#setup--build)
  - [1. PC Visualizer](#1-pc-visualizer)
  - [2. ESP32 Firmware](#2-esp32-firmware)
- [Usage](#usage)
  - [Live Mode](#live-mode)
  - [Recording Teacher & Student](#recording-teacher--student)
  - [Playback & Comparison](#playback--comparison)
  - [Keyboard Controls Summary](#keyboard-controls-summary)
- [CSV File Format](#csv-file-format)
- [Sensor‑to‑Bone Mapping](#sensor‑to‑bone-mapping)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## System Architecture

ESP32 Nodes (10x)
│
├─ MPU6050 DMP → Quaternion
├─ UDP packet [ID(1) + qw,qx,qy,qz(16)]
└─ Sent to PC IP:5005
│
▼
PC (Ubuntu)
├─ UdpReceiver (background thread)
├─ MotionRecorder (live / CSV recording)
├─ Renderer (OpenGL 3.3 Core, GPU skinning)
└─ Side‑by‑side comparison window
text


The visualizer uses **Assimp** to load a `human.glb` rigged model, **GLFW** for
windowing, **GLAD** for OpenGL loading, and performs **forward kinematics (FK)**
on the quaternion data before skinning the mesh on the GPU.

---

## Folder Structure

ESP32-Body-Tracking-System/
├── firmware/ # ESP32 PlatformIO project
│ ├── platformio.ini
│ └── src/
│ └── main.cpp
├── visualizer/ # PC OpenGL application
│ ├── assets/models/
│ │ └── human.glb # ← place your Mixamo GLB model here
│ ├── include/
│ │ ├── MathTypes.h
│ │ ├── Model.h
│ │ ├── MotionRecorder.h
│ │ ├── Network.h
│ │ ├── Renderer.h
│ │ └── Skeleton.h
│ ├── src/
│ │ ├── main.cpp
│ │ ├── Model.cpp
│ │ ├── MotionRecorder.cpp
│ │ ├── Network.cpp
│ │ └── Renderer.cpp
│ ├── CMakeLists.txt # (optional, use top‑level one)
│ └── recordings/ # created at runtime, stores CSV files
├── third_party/glad/ # GLAD OpenGL loader
│ ├── include/glad/gl.h
│ ├── include/KHR/khrplatform.h
│ └── src/gl.c
├── CMakeLists.txt # Top‑level build file
└── README.md
text


---

## Hardware Requirements

- **10 × ESP32‑C3** (or any ESP32) with MPU‑6050 (GY‑521) breakout boards.
- Each sensor must be securely attached to the performer's body at the following
  locations (see mapping table below).
- A WiFi router (same network for all nodes and the PC).

---

## Software Dependencies (PC)

Install these on your Ubuntu machine:

```bash
sudo apt update
sudo apt install build-essential cmake libglfw3-dev libassimp-dev \
                 libgl1-mesa-dev libx11-dev libxrandr-dev libxi-dev \
                 libxinerama-dev libxcursor-dev

Python (optional, for the quick visualizer):
bash

pip install matplotlib

Setup & Build
1. PC Visualizer

    Place your GLB model
    Put a rigged humanoid model (e.g., from Mixamo) into visualizer/assets/models/human.glb.

    Generate GLAD (if not already present)
    The third_party/glad/ folder is provided with this repository. If you need to
    regenerate it, run:
    bash

    pip install glad2
    python -m glad --api gl:core=3.3 --out-path third_party/glad c

    Then move the generated include/glad/gl.h, include/KHR/khrplatform.h,
    and src/gl.c into the corresponding directories.

    Build
    bash

    cd ESP32-Body-Tracking-System
    mkdir build && cd build
    cmake ..
    make -j$(nproc)

    Run
    bash

    ./body_tracking_visualizer

2. ESP32 Firmware

    Install PlatformIO.

    Open the firmware/ folder in VS Code (or use the CLI).

    Edit src/main.cpp:

        Set your WIFI_SSID, WIFI_PASSWORD, and PC_IP (the Ubuntu machine's IP).

        Set a unique SENSOR_ID for each board (0‑9).

    Connect the ESP32 via USB and click Upload.

    Repeat for all 10 boards, changing only SENSOR_ID.

Usage
Live Mode

    Start the visualiser first.

    Power on the ESP32 nodes; they will automatically connect and begin streaming.

    The human model will animate in real time, following the performer's movements.

Recording Teacher & Student

    T – start recording the teacher.

    S – start recording the student.

    Perform the dance / movement.

    R – stop recording and save the CSV files.
    They will be written to visualizer/recordings/teacher.csv and
    visualizer/recordings/student.csv.

Playback & Comparison

    P – load both CSV files and begin side‑by‑side playback.

    The teacher appears on the left (blue tint, cyan skeleton), the student on
    the right (red tint, red skeleton).

    Use SPACE to pause/resume, ← / → to seek.

Keyboard Controls Summary
Key	Action
T	Start/stop recording teacher
S	Start/stop recording student
R	Stop current recording and save CSV
P	Load both recordings and play comparison
SPACE	Pause / resume playback
← / →	Seek backward / forward 1 second
ESC	Quit
CSV File Format

Each recording is a plain CSV file with the following columns:
text

timestamp,qw0,qx0,qy0,qz0,qw1,qx1,qy1,qz1, ... ,qw9,qx9,qy9,qz9
0.000,1.0,0.0,0.0,0.0, ...
0.016,0.999,0.001,0.002,0.003, ...
...

    timestamp – seconds since recording start.

    qwX,qxX,qyX,qzX – quaternion for sensor X (0‑9).

These files can be opened in Excel, Python, or any spreadsheet for further analysis.
Sensor‑to‑Bone Mapping
Sensor ID	Body Part	Bone Name (Mixamo)
0	Hips (root)	mixamorig:Hips
1	Chest / Spine	mixamorig:Spine1
2	Head	mixamorig:Head
3	Left upper arm	mixamorig:LeftArm
4	Left forearm	mixamorig:LeftForeArm
5	Right upper arm	mixamorig:RightArm
6	Right forearm	mixamorig:RightForeArm
7	Left upper leg	mixamorig:LeftUpLeg
8	Right upper leg	mixamorig:RightUpLeg
9	(Spare) Left foot	mixamorig:LeftFoot

If your GLB model uses different bone names, update visualizer/include/Skeleton.h accordingly.
Troubleshooting

    Model not visible / black screen
    Ensure human.glb is in visualizer/assets/models/. Run the program from the
    project root (so the relative path works). If the model appears tiny or huge,
    adjust scale in Renderer::renderLive() – use 0.01 for centimetre‑scale
    models (Mixamo default) and 1.0 for metre‑scale.

    UDP not receiving data
    Check that the ESP32 nodes are on the same WiFi network and that PC_IP is
    correct (use hostname -I on Ubuntu). Also verify the PC firewall allows UDP
    port 5005.

    Shaders compile errors
    The shaders are embedded in Renderer.cpp. If you see error messages, check the
    console output – it will print the exact GLSL error.

    GLAD not found
    Make sure the files third_party/glad/src/gl.c, include/glad/gl.h, and
    include/KHR/khrplatform.h exist and match the provided content.

License

This project is provided for educational and research purposes. You are free to
use, modify, and distribute it.

Enjoy capturing and comparing Bharatanatyam movements!
text


Now you have the complete project – firmware, visualizer, GLAD loader, build system, and documentation.

