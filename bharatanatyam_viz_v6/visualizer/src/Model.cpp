// glad MUST be the very first include
#include <glad/glad.h>
#include "Model.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <algorithm>
#include <cfloat>

namespace mocap {

static Mat4 ai2mat(const aiMatrix4x4& a){
    Mat4 m;
    m.m[0]=a.a1; m.m[4]=a.a2; m.m[8] =a.a3; m.m[12]=a.a4;
    m.m[1]=a.b1; m.m[5]=a.b2; m.m[9] =a.b3; m.m[13]=a.b4;
    m.m[2]=a.c1; m.m[6]=a.c2; m.m[10]=a.c3; m.m[14]=a.c4;
    m.m[3]=a.d1; m.m[7]=a.d2; m.m[11]=a.d3; m.m[15]=a.d4;
    return m;
}

bool Model::loadFromFile(const std::string& path){
    Assimp::Importer imp;
    const aiScene* sc=imp.ReadFile(path,
        aiProcess_Triangulate|aiProcess_GenSmoothNormals|
        aiProcess_LimitBoneWeights|aiProcess_JoinIdenticalVertices);
    if(!sc||!sc->mRootNode){
        std::cerr<<"Assimp: "<<imp.GetErrorString()<<std::endl;
        return false;
    }
    buildBoneList(sc);
    buildParentIndices(sc->mRootNode,-1);
    processNode(sc->mRootNode,sc);
    indexCount_=(GLsizei)indices.size();

    bboxMin={FLT_MAX,FLT_MAX,FLT_MAX};
    bboxMax={-FLT_MAX,-FLT_MAX,-FLT_MAX};
    for(auto& v:vertices){
        bboxMin.x=std::min(bboxMin.x,v.position.x);
        bboxMin.y=std::min(bboxMin.y,v.position.y);
        bboxMin.z=std::min(bboxMin.z,v.position.z);
        bboxMax.x=std::max(bboxMax.x,v.position.x);
        bboxMax.y=std::max(bboxMax.y,v.position.y);
        bboxMax.z=std::max(bboxMax.z,v.position.z);
    }
    std::cout<<"Model: "<<vertices.size()<<" verts, "
             <<indices.size()/3<<" tris, "<<bones.size()<<" bones\n";
    return true;
}

void Model::buildBoneList(const aiScene* sc){
    for(unsigned mi=0;mi<sc->mNumMeshes;mi++){
        aiMesh* mesh=sc->mMeshes[mi];
        for(unsigned bi=0;bi<mesh->mNumBones;bi++){
            aiBone* ab=mesh->mBones[bi];
            std::string name=ab->mName.C_Str();
            if(!boneNameToIdx_.count(name)){
                int idx=(int)bones.size();
                boneNameToIdx_[name]=idx;
                BoneInfo info;
                info.name=name;
                info.invBindMatrix=ai2mat(ab->mOffsetMatrix);
                bones.push_back(info);
            }
        }
    }
}

void Model::buildParentIndices(const aiNode* node,int parentBoneIdx){
    std::string name=node->mName.C_Str();
    int myIdx=parentBoneIdx;
    auto it=boneNameToIdx_.find(name);
    if(it!=boneNameToIdx_.end()){
        bones[it->second].parentIdx=parentBoneIdx;
        bones[it->second].localBindPose=ai2mat(node->mTransformation);
        myIdx=it->second;
    }
    for(unsigned i=0;i<node->mNumChildren;i++)
        buildParentIndices(node->mChildren[i],myIdx);
}

void Model::processNode(const aiNode* node,const aiScene* sc){
    for(unsigned i=0;i<node->mNumMeshes;i++)
        processMesh(sc->mMeshes[node->mMeshes[i]]);
    for(unsigned i=0;i<node->mNumChildren;i++)
        processNode(node->mChildren[i],sc);
}

void Model::processMesh(aiMesh* mesh){
    unsigned base=(unsigned)vertices.size();
    for(unsigned i=0;i<mesh->mNumVertices;i++){
        SkinnedVertex v;
        v.position={mesh->mVertices[i].x,mesh->mVertices[i].y,mesh->mVertices[i].z};
        if(mesh->HasNormals())
            v.normal={mesh->mNormals[i].x,mesh->mNormals[i].y,mesh->mNormals[i].z};
        if(mesh->HasTextureCoords(0)){
            v.uv[0]=mesh->mTextureCoords[0][i].x;
            v.uv[1]=mesh->mTextureCoords[0][i].y;
        }
        vertices.push_back(v);
    }
    for(unsigned bi=0;bi<mesh->mNumBones;bi++){
        aiBone* ab=mesh->mBones[bi];
        int boneIdx=boneNameToIdx_.at(ab->mName.C_Str());
        for(unsigned wi=0;wi<ab->mNumWeights;wi++){
            unsigned vid=base+ab->mWeights[wi].mVertexId;
            float w=ab->mWeights[wi].mWeight;
            for(int s=0;s<4;s++){
                if(vertices[vid].boneWgt[s]==0.f){
                    vertices[vid].boneIdx[s]=boneIdx;
                    vertices[vid].boneWgt[s]=w;
                    break;
                }
            }
        }
    }
    for(unsigned i=0;i<mesh->mNumFaces;i++)
        for(unsigned j=0;j<mesh->mFaces[i].mNumIndices;j++)
            indices.push_back(base+mesh->mFaces[i].mIndices[j]);
}

void Model::uploadToGPU(){
    glGenVertexArrays(1,&vao);
    glGenBuffers(1,&vbo);
    glGenBuffers(1,&ebo);
    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER,vbo);
    glBufferData(GL_ARRAY_BUFFER,
        (GLsizei)(vertices.size()*sizeof(SkinnedVertex)),
        vertices.data(),GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
        (GLsizei)(indices.size()*sizeof(unsigned int)),
        indices.data(),GL_STATIC_DRAW);

    GLsizei stride=(GLsizei)sizeof(SkinnedVertex);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,stride,
        (void*)offsetof(SkinnedVertex,position));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,stride,
        (void*)offsetof(SkinnedVertex,normal));

    glEnableVertexAttribArray(2);
    glVertexAttribIPointer(2,4,GL_INT,stride,
        (void*)offsetof(SkinnedVertex,boneIdx));

    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3,4,GL_FLOAT,GL_FALSE,stride,
        (void*)offsetof(SkinnedVertex,boneWgt));

    glBindVertexArray(0);
    std::cout<<"GPU upload done (VAO="<<vao<<")\n";
}

void Model::draw() const {
    if(!indexCount_) return;
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES,indexCount_,GL_UNSIGNED_INT,0);
    glBindVertexArray(0);
}

} // namespace mocap
