#pragma once
#include <array>
#include <string>

namespace mocap {

constexpr int kNumSensors = 10;

struct SensorBoneMap {
    int         sensorId;
    const char* boneName;
};

// 10 IMU sensors → 10 Mixamo bones
// Bone names must match the human.glb exactly (mixamorig: prefix)
inline const std::array<SensorBoneMap, kNumSensors> kSensorMap = {{
    { 0,  "mixamorig:Hips"        },  // Pelvis / root
    { 1,  "mixamorig:Spine1"      },  // Mid chest
    { 2,  "mixamorig:Head"        },  // Head
    { 3,  "mixamorig:RightArm"    },  // Right upper arm
    { 4,  "mixamorig:RightForeArm"},  // Right forearm
    { 5,  "mixamorig:LeftArm"     },  // Left upper arm
    { 6,  "mixamorig:LeftForeArm" },  // Left forearm
    { 7,  "mixamorig:RightUpLeg"  },  // Right thigh
    { 8,  "mixamorig:LeftUpLeg"   },  // Left thigh
    { 9,  "mixamorig:RightLeg"    },  // Right shin
}};

} // namespace mocap
