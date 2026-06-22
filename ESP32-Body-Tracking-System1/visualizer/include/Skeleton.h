#pragma once
#include <array>
#include <string>

namespace mocap {

struct BoneMapping {
    int sensorId;          // which sensor drives this bone (-1 = FK only)
    const char* boneName;  // must match a joint name in the GLB
    int parentIndex;       // index into this array
};

// Order must match the bone order in the GLB (we will resolve by name)
inline const std::array<BoneMapping, 11> kSkeletonBones = {{
    // id, name, parent index
    { 0, "mixamorig:Hips",          -1 },
    { 1, "mixamorig:Spine1",         0 },
    { 2, "mixamorig:Head",           1 },
    { 3, "mixamorig:LeftArm",        1 },
    { 4, "mixamorig:LeftForeArm",    3 },
    { 5, "mixamorig:RightArm",       1 },
    { 6, "mixamorig:RightForeArm",   5 },
    { 7, "mixamorig:LeftUpLeg",      0 },
    { 8, "mixamorig:RightUpLeg",     0 },
    { 9, "mixamorig:LeftFoot",       7 },
    {10, "mixamorig:RightFoot",      8 }
}};

} // namespace mocap