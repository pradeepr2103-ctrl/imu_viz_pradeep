#include "Renderer.h"
#include "Skeleton.h"
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cmath>

namespace mocap {

// ── Shader Sources ────────────────────────────────────────────────────────────

// Skinned mesh vertex shader — up to 128 bones, 4 weights per vertex
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
    // Build skinning matrix from up to 4 bone influences
    mat4 skin = mat4(0.0);
    float totalW = 0.0;
    for(int i=0;i<4;i++){
        float w = aBoneWgt[i];
        if(w > 0.0){
            skin += w * uBones[aBoneIdx[i]];
            totalW += w;
        }
    }
    // Fallback to identity if no weights (un-skinned mesh part)
    if(totalW < 0.001)
        skin = mat4(1.0);
    else if(totalW < 0.999)
        skin /= totalW;   // renormalize

    vec4 skinnedPos    = skin * vec4(aPos, 1.0);
    vec4 skinnedNormal = skin * vec4(aNormal, 0.0);

    vWorldPos    = vec3(uModel * skinnedPos);
    vWorldNormal = normalize(mat3(uModel) * skinnedNormal.xyz);

    gl_Position  = uProjection * uView * uModel * skinnedPos;
}
)GLSL";

// Fragment shader — Blinn-Phong lighting, tint colour, rim light for body feel
static const char* kFragSkin = R"GLSL(
#version 330 core
in vec3 vWorldPos;
in vec3 vWorldNormal;

uniform vec3 uTint;         // body tint (warm skin / blue / red)
uniform vec3 uLightDir;     // key light direction (world space, normalised)
uniform float uAmbient;
uniform float uDiffuse;
uniform float uSpec;

out vec4 fragColor;

void main(){
    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(uLightDir);
    vec3 V = normalize(-vWorldPos); // approx eye direction

    // Ambient
    vec3 col = uTint * uAmbient;

    // Diffuse (half-Lambert for softer look)
    float diff = max(dot(N, L), 0.0);
    diff = diff * 0.5 + 0.5;   // remap [0,1] → [0.5,1] — half-Lambert
    col += uTint * uDiffuse * diff;

    // Blinn-Phong specular
    vec3 H = normalize(L + V);
    float spec = pow(max(dot(N, H), 0.0), 64.0);
    col += vec3(1.0) * uSpec * spec;

    // Rim light (subtle warm rim from back)
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
    glfwWindowHint(GLFW_SAMPLES,4);  // 4x MSAA

    window_ = glfwCreateWindow(1280,720,"Bharatanatyam Comparison",nullptr,nullptr);
    if(!window_){ std::cerr<<"glfwCreateWindow failed\n"; glfwTerminate(); return; }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    if(!gladLoadGL((GLADloadfunc)glfwGetProcAddress)){
        std::cerr<<"gladLoadGL failed\n"; return;
    }
    std::cout<<"OpenGL "<<glGetString(GL_VERSION)<<std::endl;

    // Load model
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
    // Back-face culling — comment out if model has inside-out faces
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
}

Renderer::~Renderer(){
    if(model_.vao) glDeleteVertexArrays(1,&model_.vao);
    if(model_.vbo) glDeleteBuffers(1,&model_.vbo);
    if(model_.ebo) glDeleteBuffers(1,&model_.ebo);
    if(skinnedProg_) glDeleteProgram(skinnedProg_);
    if(window_) glfwDestroyWindow(window_);
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

// FK: boneWorld[i] = boneWorld[parent] * localRot[i]
// skinMat[i]  = boneWorld[i] * invBindMatrix[i]
void Renderer::computeFK(const std::array<Quat,kNumSensors>& pose){
    int nb=(int)model_.bones.size();

    // Start each bone with its bind-pose local transform
    std::vector<Mat4> local(nb);
    for(int i=0;i<nb;i++)
        local[i] = model_.bones[i].localBindPose;

    // Override with sensor quaternions where available
    for(int s=0;s<kNumSensors;s++){
        int b=sensorToBone_[s];
        if(b<0) continue;
        local[b] = Mat4::fromQuat(pose[s]);
    }

    // Propagate world transforms root → leaf
    for(int i=0;i<nb;i++){
        int p=model_.bones[i].parentIdx;
        if(p<0)
            boneWorld_[i]=local[i];
        else
            boneWorld_[i]=boneWorld_[p]*local[i];

        // Final skinning matrix = world * invBind
        skinMat_[i]=boneWorld_[i]*model_.bones[i].invBindMatrix;
    }
}

void Renderer::uploadBones(){
    glUseProgram(skinnedProg_);
    int nb=std::min((int)skinMat_.size(),128);
    // Upload as array — many drivers support bulk mat4 array upload
    glUniformMatrix4fv(
        glGetUniformLocation(skinnedProg_,"uBones"),
        nb, GL_FALSE, skinMat_[0].m);
}

// Build view matrix with orbit camera
Mat4 Renderer::buildView(float distZ, float angleY) const {
    Vec3 center = meshCenter();
    // Orbit around center
    float cx = sinf(angleY)*distZ;
    float cz = cosf(angleY)*distZ;
    Vec3 eye = {center.x+cx, center.y+0.2f, center.z+cz};
    return Mat4::lookAt(eye, center, {0,1,0});
}

Mat4 Renderer::buildProj(float aspect) const {
    return Mat4::perspective(45.f*3.14159f/180.f, aspect, 0.01f, 100.f);
}

Vec3 Renderer::meshCenter() const {
    return {
        (model_.bboxMin.x+model_.bboxMax.x)*0.5f,
        (model_.bboxMin.y+model_.bboxMax.y)*0.5f,
        (model_.bboxMin.z+model_.bboxMax.z)*0.5f
    };
}

void Renderer::drawMesh(const Mat4& proj, const Mat4& view,
                        const Mat4& modelMat, const Vec3& tint){
    glUseProgram(skinnedProg_);
    glUniformMatrix4fv(uMVP_,  1,GL_FALSE,proj.m);
    glUniformMatrix4fv(uView_, 1,GL_FALSE,view.m);
    glUniformMatrix4fv(uModel_,1,GL_FALSE,modelMat.m);
    glUniform3f(uTint_,  tint.x,tint.y,tint.z);
    // Key light from upper-front-right in world space
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

    float aspect=(float)W/(float)H;
    Mat4 proj = buildProj(aspect);
    Mat4 view = buildView(3.5f, 0.f);
    Mat4 mdl  = Mat4::identity();

    // Warm skin tone — like the T-pose preview
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
    float aspect = (float)halfW/(float)H;
    Mat4 proj = buildProj(aspect);
    Mat4 mdl  = Mat4::identity();

    // ── LEFT: Teacher — warm blue-white tint ───────────────────────────────
    glViewport(0,0,halfW,H);
    glScissor(0,0,halfW,H);
    glEnable(GL_SCISSOR_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);

    computeFK(teacherPose);
    Mat4 viewT = buildView(3.5f, -0.15f);   // slight left angle
    drawMesh(proj, viewT, mdl, {0.60f, 0.75f, 1.00f});

    // ── RIGHT: Student — warm red tint ────────────────────────────────────
    glViewport(halfW,0,W-halfW,H);
    glScissor(halfW,0,W-halfW,H);
    glClear(GL_DEPTH_BUFFER_BIT);

    computeFK(studentPose);
    Mat4 viewS = buildView(3.5f, 0.15f);    // slight right angle
    drawMesh(proj, viewS, mdl, {1.00f, 0.50f, 0.45f});

    glDisable(GL_SCISSOR_TEST);

    // ── Divider ────────────────────────────────────────────────────────────
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
    // HUD is printed to stdout — for on-screen text you need freetype or a
    // bitmap font atlas. For now we print status to the window title.
    char title[256];
    if(isCompare && playing){
        std::snprintf(title,sizeof(title),
            "Bharatanatyam Comparison  [PLAYING %.1fs]  Space=pause  </>= scrub",
            playbackTime);
    } else if(recording){
        std::snprintf(title,sizeof(title),
            "Bharatanatyam Comparison  [● REC]  R=stop  %s", status.c_str());
    } else {
        std::snprintf(title,sizeof(title),
            "Bharatanatyam Comparison  T=rec teacher  S=rec student  P=compare  ESC=quit");
    }
    glfwSetWindowTitle(window_, title);
}

} // namespace mocap
