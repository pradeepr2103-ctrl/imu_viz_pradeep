#pragma once
#include <vector>
#include <string>
#include <assimp/scene.h>
#include "MathTypes.h"

namespace mocap {

struct SkinnedVertex {
    Vec3  position;
    Vec3  normal;
    int   boneIndices[4] = {0,0,0,0};
    float boneWeights[4] = {0,0,0,0};
};

struct BoneInfo {
    std::string name;
    Mat4        inverseBindMatrix;
    int         parentBoneIndex = -1;
};

class Model {
public:
    bool loadFromFile(const std::string& path);
    void draw() const;
    unsigned int indexCount() const { return indexCount_; }
    const std::vector<BoneInfo>& bones() const { return bones_; }

    unsigned int vao = 0, vbo = 0, ebo = 0;
    std::vector<SkinnedVertex> vertices;
    std::vector<unsigned int> indices;

private:
    unsigned int indexCount_ = 0;
    std::vector<BoneInfo> bones_;

    void processNode(const aiNode* node, const aiScene* scene);
    void processMesh(aiMesh* mesh);
    void extractBones(aiMesh* mesh);
};

} // namespace mocap