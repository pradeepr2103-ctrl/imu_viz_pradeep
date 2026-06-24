// glad MUST be the very first include
#include <glad/glad.h>
#include "Renderer.h"
#include "Skeleton.h"
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cmath>

namespace mocap {

// ── Shaders ───────────────────────────────────────────────────────────────────
static const char* kVS = R"GLSL(
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
out vec3 vNormal;

void main(){
    mat4 skin = mat4(0.0);
    float tw  = 0.0;
    for(int i=0;i<4;i++){
        float w=aBoneWgt[i];
        if(w>0.0){ skin+=w*uBones[aBoneIdx[i]]; tw+=w; }
    }
    if(tw<0.001) skin=mat4(1.0);
    else if(tw<0.999) skin/=tw;

    vec4 sp  = skin * vec4(aPos,1.0);
    vec4 sn  = skin * vec4(aNormal,0.0);
    vWorldPos = vec3(uModel*sp);
    vNormal   = normalize(mat3(uModel)*sn.xyz);
    gl_Position = uProjection*uView*uModel*sp;
}
)GLSL";

static const char* kFS = R"GLSL(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;

uniform vec3  uTint;
uniform vec3  uLightDir;
uniform float uAmbient;
uniform float uDiffuse;
uniform float uSpec;

out vec4 fragColor;

void main(){
    vec3 N=normalize(vNormal);
    vec3 L=normalize(uLightDir);
    vec3 V=normalize(-vWorldPos);
    vec3 col=uTint*uAmbient;
    float diff=max(dot(N,L),0.0)*0.5+0.5;
    col+=uTint*uDiffuse*diff;
    vec3 H=normalize(L+V);
    float spec=pow(max(dot(N,H),0.0),64.0);
    col+=vec3(1.0)*uSpec*spec;
    float rim=pow(1.0-max(dot(N,V),0.0),4.0);
    col+=uTint*0.12*rim;
    fragColor=vec4(col,1.0);
}
)GLSL";

// ── Flat-colour shader for sensor axis lines ──────────────────────────────────
static const char* kLineVS = R"GLSL(
#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main(){ gl_Position = uMVP * vec4(aPos,1.0); }
)GLSL";

static const char* kLineFS = R"GLSL(
#version 330 core
uniform vec3 uColor;
out vec4 fragColor;
void main(){ fragColor = vec4(uColor,1.0); }
)GLSL";

static GLuint compileShader(GLenum type,const char* src){
    GLuint s=glCreateShader(type);
    glShaderSource(s,1,&src,nullptr);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s,GL_COMPILE_STATUS,&ok);
    if(!ok){char log[1024];glGetShaderInfoLog(s,1024,nullptr,log);
            std::cerr<<"Shader error:\n"<<log<<"\n";}
    return s;
}
static GLuint linkProg(const char* vs,const char* fs){
    GLuint v=compileShader(GL_VERTEX_SHADER,vs);
    GLuint f=compileShader(GL_FRAGMENT_SHADER,fs);
    GLuint p=glCreateProgram();
    glAttachShader(p,v); glAttachShader(p,f); glLinkProgram(p);
    GLint ok; glGetProgramiv(p,GL_LINK_STATUS,&ok);
    if(!ok){char log[1024];glGetProgramInfoLog(p,1024,nullptr,log);
            std::cerr<<"Link error:\n"<<log<<"\n";}
    glDeleteShader(v); glDeleteShader(f);
    return p;
}

// ── Renderer ─────────────────────────────────────────────────────────────────
Renderer::Renderer(const std::string& glbPath){
    if(!glfwInit()){std::cerr<<"glfwInit failed\n";return;}
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,3);
    glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES,4);

    window_=glfwCreateWindow(1280,720,"IMU Visualizer",nullptr,nullptr);
    if(!window_){std::cerr<<"Window failed\n";glfwTerminate();return;}
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
        std::cerr<<"gladLoadGL failed\n";return;
    }
    std::cout<<"OpenGL "<<glGetString(GL_VERSION)<<"\n";

    if(!model_.loadFromFile(glbPath))
        std::cerr<<"Failed to load: "<<glbPath<<"\n";
    model_.uploadToGPU();

    int nb=(int)model_.bones.size();
    boneWorld_.resize(nb);
    skinMat_.resize(nb);

    setupShaders();
    buildSensorMap();
    setupAxisVAO();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
}

Renderer::~Renderer(){
    if(model_.vao){glDeleteVertexArrays(1,&model_.vao);}
    if(model_.vbo){glDeleteBuffers(1,&model_.vbo);}
    if(model_.ebo){glDeleteBuffers(1,&model_.ebo);}
    if(skinnedProg_) glDeleteProgram(skinnedProg_);
    if(lineProg_)    glDeleteProgram(lineProg_);
    if(axisVAO_)     glDeleteVertexArrays(1,&axisVAO_);
    if(axisVBO_)     glDeleteBuffers(1,&axisVBO_);
    if(window_) glfwDestroyWindow(window_);
    glfwTerminate();
}

void Renderer::setupShaders(){
    skinnedProg_=linkProg(kVS,kFS);
    glUseProgram(skinnedProg_);
    uProj_    =glGetUniformLocation(skinnedProg_,"uProjection");
    uView_    =glGetUniformLocation(skinnedProg_,"uView");
    uModel_   =glGetUniformLocation(skinnedProg_,"uModel");
    uBones_   =glGetUniformLocation(skinnedProg_,"uBones");
    uTint_    =glGetUniformLocation(skinnedProg_,"uTint");
    uLightDir_=glGetUniformLocation(skinnedProg_,"uLightDir");
    uAmbient_ =glGetUniformLocation(skinnedProg_,"uAmbient");
    uDiffuse_ =glGetUniformLocation(skinnedProg_,"uDiffuse");
    uSpec_    =glGetUniformLocation(skinnedProg_,"uSpec");
    glUseProgram(0);

    lineProg_=linkProg(kLineVS,kLineFS);
    uLineMVP_  =glGetUniformLocation(lineProg_,"uMVP");
    uLineColor_=glGetUniformLocation(lineProg_,"uColor");
}

void Renderer::setupAxisVAO(){
    // 3 axes: X(red) Y(green) Z(blue), each 2 vertices (origin -> tip)
    float axisVerts[18]={
        0,0,0,  1,0,0,  // X
        0,0,0,  0,1,0,  // Y
        0,0,0,  0,0,1   // Z
    };
    glGenVertexArrays(1,&axisVAO_);
    glGenBuffers(1,&axisVBO_);
    glBindVertexArray(axisVAO_);
    glBindBuffer(GL_ARRAY_BUFFER,axisVBO_);
    glBufferData(GL_ARRAY_BUFFER,sizeof(axisVerts),axisVerts,GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,12,nullptr);
    glBindVertexArray(0);
}

void Renderer::buildSensorMap(){
    for(int i=0;i<kNumSensors;i++) sensorToBone_[i]=-1;
    for(auto& sm:kSensorMap){
        for(int b=0;b<(int)model_.bones.size();b++){
            if(model_.bones[b].name==sm.boneName){
                sensorToBone_[sm.sensorId]=b;
                std::cout<<"Sensor "<<sm.sensorId<<" -> bone["<<b<<"] "<<sm.boneName<<"\n";
                break;
            }
        }
        if(sensorToBone_[sm.sensorId]==-1)
            std::cerr<<"WARNING: bone not found: "<<sm.boneName<<"\n";
    }
}

void Renderer::computeFK(const std::array<Quat,kNumSensors>& pose){
    int nb=(int)model_.bones.size();
    std::vector<Mat4> local(nb);
    for(int i=0;i<nb;i++) local[i]=model_.bones[i].localBindPose;
    for(int s=0;s<kNumSensors;s++){
        int b=sensorToBone_[s];
        if(b>=0) local[b]=Mat4::fromQuat(pose[s]);
    }
    for(int i=0;i<nb;i++){
        int p=model_.bones[i].parentIdx;
        boneWorld_[i]=(p<0)?local[i]:boneWorld_[p]*local[i];
        skinMat_[i]=boneWorld_[i]*model_.bones[i].invBindMatrix;
    }
}

void Renderer::uploadBones(){
    int nb=std::min((int)skinMat_.size(),128);
    glUniformMatrix4fv(uBones_,nb,GL_FALSE,skinMat_[0].m);
}

Vec3 Renderer::meshCenter() const {
    return {(model_.bboxMin.x+model_.bboxMax.x)*0.5f,
            (model_.bboxMin.y+model_.bboxMax.y)*0.5f,
            (model_.bboxMin.z+model_.bboxMax.z)*0.5f};
}

float Renderer::autoDistance() const {
    float dx = model_.bboxMax.x - model_.bboxMin.x;
    float dy = model_.bboxMax.y - model_.bboxMin.y;
    float dz = model_.bboxMax.z - model_.bboxMin.z;
    float maxDim = dx > dy ? (dx > dz ? dx : dz) : (dy > dz ? dy : dz);
    return maxDim * 1.4f;
}

Mat4 Renderer::buildView(float dist,float angleY) const {
    Vec3 c=meshCenter();
    Vec3 target = {c.x, c.y, c.z};
    Vec3 eye={c.x+sinf(angleY)*dist, c.y, c.z+cosf(angleY)*dist};
    return Mat4::lookAt(eye,target,{0,1,0});
}

Mat4 Renderer::buildProj(float aspect) const {
    float dx = model_.bboxMax.x - model_.bboxMin.x;
    float dy = model_.bboxMax.y - model_.bboxMin.y;
    float dz = model_.bboxMax.z - model_.bboxMin.z;
    float maxDim = dx > dy ? (dx > dz ? dx : dz) : (dy > dz ? dy : dz);
    float nearP = 0.001f;
    float farP  = maxDim * 20.f;
    return Mat4::perspective(45.f*3.14159f/180.f, aspect, nearP, farP);
}

void Renderer::drawMesh(const Mat4& proj,const Mat4& view,
                        const Mat4& mdl,const Vec3& tint){
    glUseProgram(skinnedProg_);
    glUniformMatrix4fv(uProj_, 1,GL_FALSE,proj.m);
    glUniformMatrix4fv(uView_, 1,GL_FALSE,view.m);
    glUniformMatrix4fv(uModel_,1,GL_FALSE,mdl.m);
    glUniform3f(uTint_,   tint.x,tint.y,tint.z);
    glUniform3f(uLightDir_,0.6f,1.2f,0.8f);
    glUniform1f(uAmbient_, 0.25f);
    glUniform1f(uDiffuse_, 0.65f);
    glUniform1f(uSpec_,    0.15f);
    uploadBones();
    model_.draw();
}

// ── Sensor panel HUD (top-right grid, like IMU_Reconstruction reference) ──────
// Each panel shows a mini axis-orientation display driven by the sensor quaternion.
// Layout: 3 columns × 4 rows max (10 sensors). Each panel is panelPx × panelPx.

static Mat4 mat4Mul(const Mat4& a, const Mat4& b){ return a*b; }

// Build an orthographic projection for pixel-space overlay
static Mat4 ortho2D(float l,float r,float b2,float t){
    Mat4 m; std::memset(m.m,0,sizeof(m.m));
    m.m[0]=2.f/(r-l);
    m.m[5]=2.f/(t-b2);
    m.m[10]=-1.f;
    m.m[12]=-(r+l)/(r-l);
    m.m[13]=-(t+b2)/(t-b2);
    m.m[15]=1.f;
    return m;
}

// Build a simple view matrix looking slightly from above+front at origin
static Mat4 sensorView(){
    Vec3 eye={0.7f,0.7f,1.4f};
    Vec3 tgt={0,0,0};
    return Mat4::lookAt(eye,tgt,{0,1,0});
}

// Build perspective for the small panel
static Mat4 sensorProj(float aspect){
    return Mat4::perspective(50.f*3.14159f/180.f, aspect, 0.01f, 10.f);
}

void Renderer::drawSensorPanels(int winW, int winH,
                                const std::array<Quat,kNumSensors>& pose){
    // Panel sizes — right column 25% of window width
    const int panelCols = 3;
    const int panelRows = 4;   // ceil(10/3)=4
    const int margin    = 6;
    const int totalPanelAreaW = winW / 4;  // rightmost 25%
    const int panelW = (totalPanelAreaW - (panelCols+1)*margin) / panelCols;
    const int panelH = panelW;  // square

    // Sensor labels matching reference image order (row-major, left to right):
    // Row0: HIPS  L_TH  L_FA
    // Row1: CHEST R_TH  R_FA
    // Row2: -     L_SH  L_UA
    // Row3: -     R_SH  R_UA
    // We map each panel position to sensor index using kSensorMap
    struct PanelEntry { int col; int row; int sensorId; const char* label; };
    static const PanelEntry panels[] = {
        {0,0, 0, "HIPS" },
        {1,0, 7, "L_TH" },   // left thigh = sensor 8 -> LeftUpLeg
        {2,0, 6, "L_FA" },   // left forearm = sensor 6 -> LeftForeArm
        {0,1, 1, "CHEST"},
        {1,1, 9, "R_TH" },   // right thigh = sensor 9 -> RightLeg
        {2,1, 4, "R_FA" },   // right forearm = sensor 4 -> RightForeArm
        {1,2, 5, "L_SH" },   // left shoulder = sensor 5 -> LeftArm
        {2,2, 3, "L_UA" },   // right arm = sensor 3 (swap label for display)
        {1,3, 8, "R_SH" },   // right shoulder area
        {2,3, 2, "R_UA" },   // head used as right upper reference
    };
    const int numPanels = 10;

    // Panel area starts at right 25% of screen, top-aligned with small margin
    const int areaX = winW - totalPanelAreaW;
    const int areaY = 0;  // top

    glDisable(GL_CULL_FACE);
    glLineWidth(2.0f);

    Mat4 view = sensorView();

    for(int pi=0; pi<numPanels; pi++){
        auto& pe = panels[pi];
        int px = areaX + margin + pe.col*(panelW+margin);
        int py = winH - (areaY + margin + pe.row*(panelH+margin) + panelH); // flip Y

        // Dark background for this panel
        glViewport(px, py, panelW, panelH);
        glScissor( px, py, panelW, panelH);
        glEnable(GL_SCISSOR_TEST);
        glClearColor(0.08f,0.09f,0.12f,1.f);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        glDisable(GL_SCISSOR_TEST);

        // Build MVP: sensor quaternion rotates the axes
        Quat q = pose[pe.sensorId];
        Mat4 rot = Mat4::fromQuat(q);
        float aspect = (float)panelW / (float)panelH;
        Mat4 proj = sensorProj(aspect);
        Mat4 mvp  = mat4Mul(mat4Mul(proj, view), rot);

        glUseProgram(lineProg_);
        glUniformMatrix4fv(uLineMVP_,1,GL_FALSE,mvp.m);
        glBindVertexArray(axisVAO_);

        // X axis — red
        glUniform3f(uLineColor_, 0.9f, 0.2f, 0.2f);
        glDrawArrays(GL_LINES, 0, 2);
        // Y axis — green
        glUniform3f(uLineColor_, 0.2f, 0.9f, 0.2f);
        glDrawArrays(GL_LINES, 2, 2);
        // Z axis — blue (lighter, same as reference)
        glUniform3f(uLineColor_, 0.3f, 0.5f, 1.0f);
        glDrawArrays(GL_LINES, 4, 2);

        glBindVertexArray(0);
        glUseProgram(0);
    }

    // Restore full viewport
    glViewport(0,0,winW,winH);
    glEnable(GL_CULL_FACE);
    glLineWidth(1.0f);
}

// ── renderLive — model on left 75%, sensor panels top-right ──────────────────
void Renderer::renderLive(const std::array<Quat,kNumSensors>& pose){
    int W,H; glfwGetFramebufferSize(window_,&W,&H);
    glViewport(0,0,W,H);
    glClearColor(0.08f,0.08f,0.10f,1.f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    computeFK(pose);

    // Draw model in left 75%
    int modelW = W * 3/4;
    glViewport(0,0,modelW,H);
    glScissor(0,0,modelW,H);
    glEnable(GL_SCISSOR_TEST);
    glClear(GL_DEPTH_BUFFER_BIT);
    float aspect=(float)modelW/std::max(H,1);
    float dist = autoDistance();
    drawMesh(buildProj(aspect), buildView(dist, 0.f), Mat4::identity(),
             {0.90f,0.72f,0.62f});
    glDisable(GL_SCISSOR_TEST);

    // Draw sensor panels in right 25%
    glViewport(0,0,W,H);
    drawSensorPanels(W, H, pose);

    glfwSwapBuffers(window_);
    glfwPollEvents();
}

// ── renderCompare — two half-models, no sensor panels (cleaner) ───────────────
void Renderer::renderCompare(const std::array<Quat,kNumSensors>& teacherPose,
                             const std::array<Quat,kNumSensors>& studentPose){
    int W,H; glfwGetFramebufferSize(window_,&W,&H);
    int halfW=W/2;
    float aspect=(float)halfW/std::max(H,1);
    Mat4 proj=buildProj(aspect);
    Mat4 mdl =Mat4::identity();
    float dist = autoDistance();

    glClearColor(0.06f,0.06f,0.08f,1.f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

    // Teacher — left, blue tint
    glEnable(GL_SCISSOR_TEST);
    glViewport(0,0,halfW,H);
    glScissor(0,0,halfW,H);
    glClear(GL_DEPTH_BUFFER_BIT);
    computeFK(teacherPose);
    drawMesh(proj,buildView(dist,-0.15f),mdl,{0.55f,0.72f,1.00f});

    // Student — right, red tint
    glViewport(halfW,0,W-halfW,H);
    glScissor(halfW,0,W-halfW,H);
    glClear(GL_DEPTH_BUFFER_BIT);
    computeFK(studentPose);
    drawMesh(proj,buildView(dist,0.15f),mdl,{1.00f,0.48f,0.42f});

    glDisable(GL_SCISSOR_TEST);

    // Centre divider
    glViewport(0,0,W,H);
    glEnable(GL_SCISSOR_TEST);
    glScissor(halfW-1,0,2,H);
    glClearColor(0.40f,0.40f,0.45f,1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);

    glfwSwapBuffers(window_);
    glfwPollEvents();
}

void Renderer::drawHUD(bool recording,bool playing,bool isCompare,
                       double playbackTime,const std::string& status){
    // Build sensor list string for title bar
    // Shows all 10 sensors like the reference: "L_FA, R_FA, L_UA, R_UA, ..."
    const char* sensorNames[kNumSensors] = {
        "HIPS","CHEST","HEAD","R_UA","R_FA","L_UA","L_FA","R_TH","L_TH","R_SH"
    };
    char sensorStr[256]="";
    for(int i=0;i<kNumSensors;i++){
        if(i>0) strcat(sensorStr,", ");
        strcat(sensorStr,sensorNames[i]);
    }

    char title[512];
    if(isCompare&&playing)
        std::snprintf(title,sizeof(title),
            "IMU Visualizer - %s  [PLAYING %.1fs]  Space=pause  </>=scrub",
            sensorStr, playbackTime);
    else if(recording)
        std::snprintf(title,sizeof(title),
            "IMU Visualizer - %s  [● REC %s]  R=stop",
            sensorStr, status.c_str());
    else
        std::snprintf(title,sizeof(title),
            "IMU Visualizer - %s  | T=teacher  S=student  R=stop  P=compare  ESC=quit",
            sensorStr);
    glfwSetWindowTitle(window_,title);
}

} // namespace mocap
