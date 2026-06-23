#pragma once
#include <array>
#include <string>

namespace mocap {

constexpr int kNumSensors = 10;

// Maps each ESP32 sensor ID to the exact Mixamo bone name in human.glb
struct SensorBoneMap {
    int         sensorId;
    const char* boneName;
};

// 10 IMU sensors → 10 Mixamo bones
// Remap sensorId assignments to match your physical sensor placement
inline const std::array<SensorBoneMap, kNumSensors> kSensorMap = {{
    { 0,  "mixamorig:Hips"        },
    { 1,  "mixamorig:Spine1"      },
    { 2,  "mixamorig:Head"        },
    { 3,  "mixamorig:RightArm"    },
    { 4,  "mixamorig:RightForeArm"},
    { 5,  "mixamorig:LeftArm"     },
    { 6,  "mixamorig:LeftForeArm" },
    { 7,  "mixamorig:RightUpLeg"  },
    { 8,  "mixamorig:LeftUpLeg"   },
    { 9,  "mixamorig:RightLeg"    },
}};

} // namespace mocap
