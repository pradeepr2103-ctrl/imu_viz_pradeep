#include "Model.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <iostream>
#include <cstring>

namespace mocap {

bool Model::loadFromFile(const std::string& path) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(path,
        aiProcess_Triangulate | aiProcess_GenNormals);
    if (!scene || !scene->mRootNode) {
        std::cerr << "Assimp error: " << importer.GetErrorString() << std::endl;
        return false;
    }
    processNode(scene->mRootNode, scene);
    indexCount_ = (unsigned int)indices.size();
    std::cout << "Loaded " << vertices.size() << " vertices, "
              << indices.size()/3 << " triangles, "
              << bones_.size() << " bones" << std::endl;
    return true;
}

void Model::processNode(const aiNode* node, const aiScene* scene) {
    for (unsigned i = 0; i < node->mNumMeshes; i++)
        processMesh(scene->mMeshes[node->mMeshes[i]]);
    for (unsigned i = 0; i < node->mNumChildren; i++)
        processNode(node->mChildren[i], scene);
}

void Model::processMesh(aiMesh* mesh) {
    for (unsigned i = 0; i < mesh->mNumVertices; i++) {
        SkinnedVertex v;
        v.position = {mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z};
        if (mesh->HasNormals())
            v.normal = {mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z};
        memset(v.boneIndices, 0, sizeof(v.boneIndices));
        memset(v.boneWeights, 0, sizeof(v.boneWeights));
        vertices.push_back(v);
    }
    for (unsigned i = 0; i < mesh->mNumFaces; i++)
        for (unsigned j = 0; j < mesh->mFaces[i].mNumIndices; j++)
            indices.push_back(mesh->mFaces[i].mIndices[j]);
    extractBones(mesh);
}

void Model::extractBones(aiMesh* mesh) {
    for (unsigned b = 0; b < mesh->mNumBones; b++) {
        aiBone* bone = mesh->mBones[b];
        std::string name = bone->mName.C_Str();
        int idx = -1;
        for (int i = 0; i < (int)bones_.size(); i++)
            if (bones_[i].name == name) { idx = i; break; }
        if (idx == -1) {
            BoneInfo bi;
            bi.name = name;
            const aiMatrix4x4& off = bone->mOffsetMatrix;
            bi.inverseBindMatrix.m[0]  = off.a1; bi.inverseBindMatrix.m[4]  = off.a2;
            bi.inverseBindMatrix.m[8]  = off.a3; bi.inverseBindMatrix.m[12] = off.a4;
            bi.inverseBindMatrix.m[1]  = off.b1; bi.inverseBindMatrix.m[5]  = off.b2;
            bi.inverseBindMatrix.m[9]  = off.b3; bi.inverseBindMatrix.m[13] = off.b4;
            bi.inverseBindMatrix.m[2]  = off.c1; bi.inverseBindMatrix.m[6]  = off.c2;
            bi.inverseBindMatrix.m[10] = off.c3; bi.inverseBindMatrix.m[14] = off.c4;
            bi.inverseBindMatrix.m[3]  = off.d1; bi.inverseBindMatrix.m[7]  = off.d2;
            bi.inverseBindMatrix.m[11] = off.d3; bi.inverseBindMatrix.m[15] = off.d4;
            idx = (int)bones_.size();
            bones_.push_back(bi);
        }
        for (unsigned w = 0; w < bone->mNumWeights; w++) {
            unsigned vertId = bone->mWeights[w].mVertexId;
            float weight = bone->mWeights[w].mWeight;
            for (int j = 0; j < 4; j++) {
                if (vertices[vertId].boneWeights[j] == 0.0f) {
                    vertices[vertId].boneIndices[j] = idx;
                    vertices[vertId].boneWeights[j] = weight;
                    break;
                }
            }
        }
    }
}

void Model::draw() const {
    if (!indexCount_) return;
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, 0);
}

} // namespace mocap