#pragma once
#include <array>
#include <string>

namespace mocap {

constexpr int kNumSensors = 10;

struct SensorBoneMap {
    int         sensorId;
    const char* boneName;
    const char* displayName;  // Short label shown in panel HUD
};

// Sensor ID → Mixamo bone mapping
// Sensor IDs match the ESP32 BOARD_NUMBER flashed to each board
inline const std::array<SensorBoneMap, kNumSensors> kSensorMap = {{
    { 0, "mixamorig:Hips",         "HIPS"  },
    { 1, "mixamorig:Spine1",       "CHEST" },
    { 2, "mixamorig:Head",         "HEAD"  },
    { 3, "mixamorig:RightArm",     "R_UA"  },
    { 4, "mixamorig:RightForeArm", "R_FA"  },
    { 5, "mixamorig:LeftArm",      "L_UA"  },
    { 6, "mixamorig:LeftForeArm",  "L_FA"  },
    { 7, "mixamorig:RightUpLeg",   "R_TH"  },
    { 8, "mixamorig:LeftUpLeg",    "L_TH"  },
    { 9, "mixamorig:RightLeg",     "R_SH"  },  // right shin
}};

} // namespace mocap
