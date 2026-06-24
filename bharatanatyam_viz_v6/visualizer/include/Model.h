#pragma once
// glad MUST come before any GL headers
#include <glad/glad.h>
#include <vector>
#include <string>
#include <unordered_map>
#include "MathTypes.h"

struct aiNode;
struct aiMesh;
struct aiScene;

namespace mocap {

struct SkinnedVertex {
    Vec3  position   = {0,0,0};
    Vec3  normal     = {0,0,0};
    float uv[2]      = {0,0};
    int   boneIdx[4] = {0,0,0,0};
    float boneWgt[4] = {0,0,0,0};
};

struct BoneInfo {
    std::string name;
    Mat4        invBindMatrix;
    int         parentIdx = -1;
    Mat4        localBindPose;
};

class Model {
public:
    bool loadFromFile(const std::string& path);
    void uploadToGPU();
    void draw() const;

    GLuint vao=0, vbo=0, ebo=0;
    std::vector<SkinnedVertex>  vertices;
    std::vector<unsigned int>   indices;
    std::vector<BoneInfo>       bones;
    Vec3 bboxMin={0,0,0}, bboxMax={0,0,0};

private:
    GLsizei indexCount_=0;
    std::unordered_map<std::string,int> boneNameToIdx_;

    void buildBoneList(const aiScene* sc);
    void buildParentIndices(const aiNode* node, int parentBoneIdx);
    void processNode(const aiNode* node, const aiScene* sc);
    void processMesh(aiMesh* mesh);
};

} // namespace mocap
