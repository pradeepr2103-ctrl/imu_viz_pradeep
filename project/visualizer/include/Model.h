#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include "MathTypes.h"

struct aiNode;
struct aiMesh;
struct aiScene;

namespace mocap {

// One vertex with up to 4 bone influences
struct SkinnedVertex {
    Vec3  position  = {0,0,0};
    Vec3  normal    = {0,0,0};
    float uv[2]     = {0,0};
    int   boneIdx[4]= {0,0,0,0};
    float boneWgt[4]= {0,0,0,0};
};

struct BoneInfo {
    std::string name;
    Mat4        invBindMatrix;      // offset matrix from Assimp
    int         parentIdx = -1;     // index into Model::bones, -1 = root
    Mat4        localBindPose;      // TRS from the node (identity default)
};

class Model {
public:
    bool loadFromFile(const std::string& path);
    void uploadToGPU();             // call AFTER GL context is alive
    void draw() const;

    // Public geometry (read by Renderer)
    unsigned int vao=0, vbo=0, ebo=0;
    std::vector<SkinnedVertex>  vertices;
    std::vector<unsigned int>   indices;
    std::vector<BoneInfo>       bones;  // indexed by skin joint order

    // Bounding box (set after load)
    Vec3 bboxMin={0,0,0}, bboxMax={0,0,0};

private:
    unsigned int indexCount_=0;
    std::unordered_map<std::string,int> boneNameToIdx_;

    void buildBoneList(const aiScene* sc);
    void buildParentIndices(const aiNode* node, int parentBoneIdx);
    void processNode(const aiNode* node, const aiScene* sc);
    void processMesh(aiMesh* mesh);
};

} // namespace mocap
