#include "Renderer.h"
#include "Skeleton.h"
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cmath>

namespace mocap {

// ── Shader Sources ────────────────────────────────────────────────────────────

static const char* kVertSkin = R"GLSL(
#version 330 core
layout(location=0) in vec3  aPos;
layout(location=1) in vec3  aNormal;
layout(location=2) in ivec4 aBoneIdx;
layout(location=3) in vec4  aBoneWgt;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uModel;
uniform mat4 uBones[128];

out vec3 vWorldPos;
out vec3 vWorldNormal;

void main(){
    mat4 skin = mat4(0.0);
    float totalW = 0.0;
    for(int i=0;i<4;i++){
        float w = aBoneWgt[i];
        if(w > 0.0){
            skin += w * uBones[aBoneIdx[i]];
            totalW += w;
        }
    }
    if(totalW < 0.001)
        skin = mat4(1.0);
    else if(totalW < 0.999)
        skin /= totalW;

    vec4 skinnedPos    = skin * vec4(aPos, 1.0);
    vec4 skinnedNormal = skin * vec4(aNormal, 0.0);

    vWorldPos    = vec3(uModel * skinnedPos);
    vWorldNormal = normalize(mat3(uModel) * skinnedNormal.xyz);

    gl_Position  = uProjection * uView * uModel * skinnedPos;
}
)GLSL";

static const char* kFragSkin = R"GLSL(
#version 330 core
in vec3 vWorldPos;
in vec3 vWorldNormal;

uniform vec3  uTint;
uniform vec3  uLightDir;
uniform float uAmbient;
uniform float uDiffuse;
uniform float uSpec;

out vec4 fragColor;

void main(){
    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(uLightDir);
    vec3 V = normalize(vec3(0,0,3) - vWorldPos);

    vec3 col = uTint * uAmbient;

    float diff = max(dot(N, L), 0.0);
    diff = diff * 0.5 + 0.5;
    col += uTint * uDiffuse * diff;

    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 64.0);
    col += vec3(1.0) * uSpec * spec;

    float rim = 1.0 - max(dot(N, V), 0.0);
    rim = pow(rim, 4.0);
    col += uTint * 0.15 * rim;

    fragColor = vec4(col, 1.0);
}
)GLSL";

// ── Helpers ───────────────────────────────────────────────────────────────────
static unsigned compileShader(GLenum type, const char* src){
    unsigned s = glCreateShader(type);
    glShaderSource(s,1,&src,nullptr);
    glCompileShader(s);
    int ok; glGetShaderiv(s,GL_COMPILE_STATUS,&ok);
    if(!ok){
        char log[1024]; glGetShaderInfoLog(s,1024,nullptr,log);
        std::cerr << "Shader compile error:\n" << log << std::endl;
    }
    return s;
}

static unsigned linkProgram(const char* vsrc, const char* fsrc){
    unsigned vs=compileShader(GL_VERTEX_SHADER,vsrc);
    unsigned fs=compileShader(GL_FRAGMENT_SHADER,fsrc);
    unsigned p=glCreateProgram();
    glAttachShader(p,vs); glAttachShader(p,fs);
    glLinkProgram(p);
    int ok; glGetProgramiv(p,GL_LINK_STATUS,&ok);
    if(!ok){
        char log[1024]; glGetProgramInfoLog(p,1024,nullptr,log);
        std::cerr << "Program link error:\n" << log << std::endl;
    }
    glDetachShader(p,vs); glDetachShader(p,fs);
    glDeleteShader(vs); glDeleteShader(fs);
    return p;
}

// ── Renderer ──────────────────────────────────────────────────────────────────
Renderer::Renderer(const std::string& glbPath){
    if(!glfwInit()){ std::cerr<<"glfwInit failed\n"; return; }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES,4);

    window_ = glfwCreateWindow(1280,720,"IMU Motion Capture",nullptr,nullptr);
    if(!window_){ std::cerr<<"glfwCreateWindow failed\n"; glfwTerminate(); return; }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    if(!gladLoadGL((GLADloadfunc)glfwGetProcAddress)){
        std::cerr<<"gladLoadGL failed\n"; return;
    }
    std::cout<<"OpenGL "<<glGetString(GL_VERSION)<<std::endl;

    if(!model_.loadFromFile(glbPath)){
        std::cerr<<"Failed to load: "<<glbPath<<std::endl;
    }
    model_.uploadToGPU();

    int nb=(int)model_.bones.size();
    boneWorld_.resize(nb);
    skinMat_.resize(nb);

    setupShaders();
    buildSensorMap();

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
}

Renderer::~Renderer(){
    if(skinnedProg_) glDeleteProgram(skinnedProg_);
    if(model_.vao)   glDeleteVertexArrays(1,&model_.vao);
    if(model_.vbo)   glDeleteBuffers(1,&model_.vbo);
    if(model_.ebo)   glDeleteBuffers(1,&model_.ebo);
    if(window_)      glfwDestroyWindow(window_);
    glfwTerminate();
}

void Renderer::setupShaders(){
    skinnedProg_ = linkProgram(kVertSkin, kFragSkin);
    glUseProgram(skinnedProg_);
    uMVP_      = glGetUniformLocation(skinnedProg_,"uProjection");
    uView_     = glGetUniformLocation(skinnedProg_,"uView");
    uModel_    = glGetUniformLocation(skinnedProg_,"uModel");
    uBones_    = glGetUniformLocation(skinnedProg_,"uBones[0]");
    uTint_     = glGetUniformLocation(skinnedProg_,"uTint");
    uLightDir_ = glGetUniformLocation(skinnedProg_,"uLightDir");
    uAmbient_  = glGetUniformLocation(skinnedProg_,"uAmbient");
    uDiffuse_  = glGetUniformLocation(skinnedProg_,"uDiffuse");
    uSpec_     = glGetUniformLocation(skinnedProg_,"uSpec");
    glUseProgram(0);
}

void Renderer::buildSensorMap(){
    for(int i=0;i<kNumSensors;i++) sensorToBone_[i]=-1;
    for(auto& sm : kSensorMap){
        for(int b=0;b<(int)model_.bones.size();b++){
            if(model_.bones[b].name == sm.boneName){
                sensorToBone_[sm.sensorId]=b;
                std::cout<<"Sensor "<<sm.sensorId<<" → bone["<<b<<"] "<<sm.boneName<<std::endl;
                break;
            }
        }
        if(sensorToBone_[sm.sensorId]==-1)
            std::cerr<<"WARNING: bone not found: "<<sm.boneName<<std::endl;
    }

    // Pre-compute bind-pose quaternions for each sensor-driven bone.
    // When an IMU sends a world-space quaternion Q_world, the correct local
    // bone rotation is:  Q_local = inverse(Q_bindWorld) * Q_world
    // where Q_bindWorld is the world-space rotation of that bone at rest.
    //
    // We compute Q_bindWorld by accumulating localBindPose down the chain.
    int nb = (int)model_.bones.size();
    std::vector<Mat4> worldBind(nb);
    for(int i=0;i<nb;i++){
        int p = model_.bones[i].parentIdx;
        if(p < 0)
            worldBind[i] = model_.bones[i].localBindPose;
        else
            worldBind[i] = worldBind[p] * model_.bones[i].localBindPose;
    }
    for(int s=0;s<kNumSensors;s++){
        int b = sensorToBone_[s];
        if(b < 0){ bindWorldQuat_[s] = {1,0,0,0}; continue; }
        bindWorldQuat_[s] = worldBind[b].toQuat();
    }
}

// ── Forward Kinematics ────────────────────────────────────────────────────────
//
// IMU sensors give absolute world-space orientations Q_world[s].
// The skinning pipeline expects local bone transforms (parent-relative).
//
// For each sensor-driven bone b:
//   1. Compute desired world rotation: Q_boneWorld = Q_world[s]
//   2. Get parent's current world rotation: Q_parentWorld (from FK chain so far)
//   3. Local rotation = inverse(Q_parentWorld) * Q_boneWorld
//   4. Rebuild local TRS: keep bind-pose translation & scale, replace rotation
//
// For sensor = identity quaternion (1,0,0,0), result should be bind pose → no deformation.
//
void Renderer::computeFK(const std::array<Quat,kNumSensors>& pose){
    int nb=(int)model_.bones.size();

    // Start with bind-pose local transforms for every bone
    std::vector<Mat4> local(nb);
    for(int i=0;i<nb;i++)
        local[i] = model_.bones[i].localBindPose;

    // Build world transforms in topological order (parents before children)
    // We need parent world quats to compute local quats for sensor-driven bones.
    // Do a single pass: since buildParentIndices walks tree top-down, bone[i]
    // always has parentIdx < i (guaranteed by DFS order).

    std::vector<Quat> worldQuat(nb);

    // First pass: compute bind-pose world quats
    for(int i=0;i<nb;i++){
        int p = model_.bones[i].parentIdx;
        Quat localQ = local[i].toQuat();
        if(p < 0)
            worldQuat[i] = localQ;
        else
            worldQuat[i] = quatNorm(quatMul(worldQuat[p], localQ));
    }

    // Apply sensor rotations
    for(int s=0;s<kNumSensors;s++){
        int b = sensorToBone_[s];
        if(b < 0) continue;

        // IMU gives world-space absolute orientation Q_world
        Quat Q_world = quatNorm(pose[s]);

        // Desired world rotation for this bone = Q_world
        // (sensor is calibrated so identity = T-pose bind orientation)
        // To convert to parent-relative:
        int p = model_.bones[b].parentIdx;
        Quat Q_parentWorld = (p < 0) ? Quat{1,0,0,0} : worldQuat[p];
        Quat Q_local = quatNorm(quatMul(quatInverse(Q_parentWorld), Q_world));

        // Rebuild local matrix: rotation from sensor, translation from bind pose
        Vec3 t = model_.bones[b].localBindPose.getTranslation();
        Vec3 sc = model_.bones[b].localBindPose.getScale();
        local[b] = Mat4::fromTRS(t, Q_local, sc);

        // Update world quat for this bone so children get correct parent
        worldQuat[b] = Q_world;

        // Propagate updated worldQuat down to children of b
        // (children must be processed after parents; since bones are in DFS order,
        //  we need to update all descendants inline)
        for(int j=b+1;j<nb;j++){
            int pj = model_.bones[j].parentIdx;
            if(pj < 0) continue;
            // Recompute worldQuat[j] from (possibly updated) parent
            Quat localQj = local[j].toQuat();
            worldQuat[j] = quatNorm(quatMul(worldQuat[pj], localQj));
        }
    }

    // Final pass: build world matrices and skinning matrices
    for(int i=0;i<nb;i++){
        int p = model_.bones[i].parentIdx;
        if(p < 0)
            boneWorld_[i] = local[i];
        else
            boneWorld_[i] = boneWorld_[p] * local[i];

        skinMat_[i] = boneWorld_[i] * model_.bones[i].invBindMatrix;
    }
}

void Renderer::uploadBones(){
    glUseProgram(skinnedProg_);
    int nb=std::min((int)skinMat_.size(),128);
    glUniformMatrix4fv(
        glGetUniformLocation(skinnedProg_,"uBones"),
        nb, GL_FALSE, skinMat_[0].m);
}

// ── Camera ────────────────────────────────────────────────────────────────────
// BBox Y raw (cm): 0 .. ~180
// modelMatrix scales by 0.01 → world Y: 0 .. 1.80
// Camera targets the mid-height of the scaled model.
Mat4 Renderer::buildView(float distZ, float angleY) const {
    // Compute actual world-space centre from scaled bbox
    float cy = (model_.bboxMin.y + model_.bboxMax.y) * 0.5f * 0.01f;
    Vec3 center = {0.0f, cy, 0.0f};
    Vec3 eye = {
        center.x + sinf(angleY) * distZ,
        center.y,
        center.z + cosf(angleY) * distZ
    };
    return Mat4::lookAt(eye, center, {0.f, 1.f, 0.f});
}

Mat4 Renderer::buildProj(float aspect) const {
    return Mat4::perspective(60.f * 3.14159f / 180.f, aspect, 0.001f, 100.f);
}

Vec3 Renderer::meshCenter() const {
    return {
        (model_.bboxMin.x + model_.bboxMax.x) * 0.5f * 0.01f,
        (model_.bboxMin.y + model_.bboxMax.y) * 0.5f * 0.01f,
        (model_.bboxMin.z + model_.bboxMax.z) * 0.5f * 0.01f
    };
}

// ── Model matrix ──────────────────────────────────────────────────────────────
// The GLB uses centimetre units (diag ~100 in invBind). Scale by 0.01 → metres.
// No rotation needed — Mixamo bind pose is Y-up in its own space.
static Mat4 modelMatrix() {
    Mat4 m = Mat4::identity();
    m.m[0]  = 0.01f;
    m.m[5]  = 0.01f;
    m.m[10] = 0.01f;
    return m;
}

void Renderer::drawMesh(const Mat4& proj, const Mat4& view,
                        const Mat4& modelMat, const Vec3& tint){
    glUseProgram(skinnedProg_);
    glUniformMatrix4fv(uMVP_,  1,GL_FALSE,proj.m);
    glUniformMatrix4fv(uView_, 1,GL_FALSE,view.m);
    glUniformMatrix4fv(uModel_,1,GL_FALSE,modelMat.m);
    glUniform3f(uTint_,  tint.x,tint.y,tint.z);
    glUniform3f(uLightDir_, 0.6f, 1.2f, 0.8f);
    glUniform1f(uAmbient_,  0.25f);
    glUniform1f(uDiffuse_,  0.65f);
    glUniform1f(uSpec_,     0.15f);
    uploadBones();
    model_.draw();
}

// ── Public render functions ───────────────────────────────────────────────────
void Renderer::renderLive(const std::array<Quat,kNumSensors>& pose){
    int W,H; glfwGetFramebufferSize(window_,&W,&H);
    glViewport(0,0,W,H);
    glClearColor(0.08f,0.08f,0.10f,1.f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    computeFK(pose);

    float aspect = (float)W / (float)H;
    Mat4 proj = buildProj(aspect);
    Mat4 view = buildView(2.5f, 0.f);
    Mat4 mdl  = modelMatrix();

    drawMesh(proj, view, mdl, {0.90f, 0.72f, 0.62f});

    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void Renderer::renderCompare(const std::array<Quat,kNumSensors>& teacherPose,
                             const std::array<Quat,kNumSensors>& studentPose){
    int W,H; glfwGetFramebufferSize(window_,&W,&H);

    glClearColor(0.06f,0.06f,0.08f,1.f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    int halfW = W/2;
    float aspect = (float)halfW / (float)H;
    Mat4 proj = buildProj(aspect);
    Mat4 mdl  = modelMatrix();

    // ── LEFT: Teacher (blue) ───────────────────────────────────────────────
    glViewport(0,0,halfW,H);
    glScissor(0,0,halfW,H);
    glEnable(GL_SCISSOR_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);
    computeFK(teacherPose);
    Mat4 viewT = buildView(2.5f, -0.15f);
    drawMesh(proj, viewT, mdl, {0.60f, 0.75f, 1.00f});

    // ── RIGHT: Student (red) ───────────────────────────────────────────────
    glViewport(halfW,0,W-halfW,H);
    glScissor(halfW,0,W-halfW,H);
    glClear(GL_DEPTH_BUFFER_BIT);
    computeFK(studentPose);
    Mat4 viewS = buildView(2.5f, 0.15f);
    drawMesh(proj, viewS, mdl, {1.00f, 0.50f, 0.45f});

    glDisable(GL_SCISSOR_TEST);

    // ── Divider line ──────────────────────────────────────────────────────
    glViewport(0,0,W,H);
    glEnable(GL_SCISSOR_TEST);
    glScissor(halfW-1,0,2,H);
    glClearColor(0.35f,0.35f,0.40f,1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);

    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void Renderer::drawHUD(bool recording, bool playing, bool isCompare,
                       double playbackTime, const std::string& status){
    char title[256];
    if(isCompare && playing){
        std::snprintf(title,sizeof(title),
            "IMU Mocap  [PLAYING %.1fs]  Space=pause  </>=scrub",
            playbackTime);
    } else if(recording){
        std::snprintf(title,sizeof(title),
            "IMU Mocap  [● REC]  R=stop  %s", status.c_str());
    } else {
        std::snprintf(title,sizeof(title),
            "IMU Mocap  T=rec teacher  S=rec student  P=compare  ESC=quit");
    }
    glfwSetWindowTitle(window_, title);
}

} // namespace mocap
