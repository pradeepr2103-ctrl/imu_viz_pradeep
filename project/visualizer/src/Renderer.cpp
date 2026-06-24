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
    // Normal CCW winding — model is +Y up after our scale matrix
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
}

void Renderer::computeFK(const std::array<Quat,kNumSensors>& pose){
    int nb=(int)model_.bones.size();

    std::vector<Mat4> local(nb);
    for(int i=0;i<nb;i++)
        local[i] = model_.bones[i].localBindPose;

    for(int s=0;s<kNumSensors;s++){
        int b=sensorToBone_[s];
        if(b<0) continue;
        // Preserve the bind-pose translation, only override rotation
        const Mat4& bind = model_.bones[b].localBindPose;
        Mat4 rot = Mat4::fromQuat(pose[s]);
        // Copy translation column from bind pose into rotation matrix
        rot.m[12] = bind.m[12];
        rot.m[13] = bind.m[13];
        rot.m[14] = bind.m[14];
        local[b] = rot;
    }

    for(int i=0;i<nb;i++){
        int p=model_.bones[i].parentIdx;
        if(p<0)
            boneWorld_[i]=local[i];
        else
            boneWorld_[i]=boneWorld_[p]*local[i];

        skinMat_[i]=boneWorld_[i]*model_.bones[i].invBindMatrix;
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
// The GLB is in centimetre space (~176 cm tall).
// We apply a uniform 0.01 scale in the model matrix → model is ~1.76 units tall.
// The Hips bone (root) is ~Y=155 cm in GLB space → after scale = ~1.55 world units.
// Feet are near Y=0 → after scale = 0. Head top ~Y=176 → 1.76.
// So world-space center ≈ (0, 0.88, 0).
// Camera sits at Z+ looking toward origin at that height.

Mat4 Renderer::buildView(float distZ, float angleY) const {
    // Fixed world-space target: mid-height of the scaled model
    Vec3 center = {0.0f, 1.3f, 0.0f};
    Vec3 eye = {
        center.x + sinf(angleY) * distZ,
        center.y,
        center.z + cosf(angleY) * distZ
    };
    return Mat4::lookAt(eye, center, {0.f, 1.f, 0.f});
}

Mat4 Renderer::buildProj(float aspect) const {
    // 60° FOV — wide enough to see full body at distance 2.5
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
// GLB is in cm. Scale uniformly by 0.01 → metres.
// The Mixamo FBX/GLB Y-axis IS the up axis in the bind pose, so no rotation needed.
// Pure uniform scale: only touch m[0], m[5], m[10]; leave all else as identity.
static Mat4 modelMatrix() {
    // Rotate 180° around X to flip Y direction (head->up)
    Mat4 rot = Mat4::identity();
    rot.m[5]  = -1.0f;  // cos(180) on Y axis
    rot.m[10] = -1.0f;  // cos(180) on Z axis
    // (other elements stay identity)

    // Uniform centimetre‑to‑metre scale
    Mat4 scale = Mat4::identity();
    scale.m[0] = scale.m[5] = scale.m[10] = 0.01f;

    // First rotate, then scale: M = Scale * Rotation
    return scale * rot;
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
    Mat4 view = buildView(3.5f, 0.f);
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

    // ── LEFT: Teacher ──────────────────────────────────────────────────────
    glViewport(0,0,halfW,H);
    glScissor(0,0,halfW,H);
    glEnable(GL_SCISSOR_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);
    computeFK(teacherPose);
    Mat4 viewT = buildView(2.5f, -0.15f);
    drawMesh(proj, viewT, mdl, {0.60f, 0.75f, 1.00f});

    // ── RIGHT: Student ────────────────────────────────────────────────────
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
