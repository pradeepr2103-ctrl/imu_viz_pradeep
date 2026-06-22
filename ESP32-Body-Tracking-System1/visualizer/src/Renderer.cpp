#include "Renderer.h"
#include "Skeleton.h"
#include <iostream>
#include <cstring>
#include <cmath>

namespace mocap {

// ── Helper: compile & link shaders from strings ─────────────────────────
static unsigned compileShader(GLenum type, const char* src) {
    unsigned s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    int ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, 512, nullptr, log);
        std::cerr << "Shader compile error: " << log << std::endl;
    }
    return s;
}

static unsigned linkProgram(const char* vsrc, const char* fsrc) {
    unsigned vs = compileShader(GL_VERTEX_SHADER, vsrc);
    unsigned fs = compileShader(GL_FRAGMENT_SHADER, fsrc);
    unsigned p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    glDetachShader(p, vs);
    glDetachShader(p, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);
    return p;
}

// ─────────────────────────────────────────────────────────────────────────
Renderer::Renderer(const std::string& glbPath) {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    window_ = glfwCreateWindow(1280, 720,
                               "Bharatanatyam Comparison",
                               nullptr, nullptr);
    glfwMakeContextCurrent(window_);
    gladLoadGL((GLADloadfunc)glfwGetProcAddress);
    glfwSwapInterval(0);

    if (!model_.loadFromFile(glbPath)) {
        std::cerr << "Failed to load model: " << glbPath << std::endl;
    }

    setupGL();

    // Bone matrices initialized to identity (bind pose)
    boneMatrices_.resize(model_.bones().size());
    for (auto& m : boneMatrices_) {
        std::memset(m.m, 0, sizeof(m.m));
        m.m[0] = m.m[5] = m.m[10] = m.m[15] = 1.0f;
    }
}

Renderer::~Renderer() {
    if (window_) {
        glfwDestroyWindow(window_);
        glfwTerminate();
    }
}

void Renderer::setupGL() {
    // ── Skinned mesh shader (GPU skinning + Blinn‑Phong) ────────────────
    const char* vertSkin = R"(#version 330 core
        layout(location = 0) in vec3 aPos;
        layout(location = 1) in vec3 aNormal;
        layout(location = 2) in ivec4 aBoneIdx;
        layout(location = 3) in vec4 aBoneWeight;

        uniform mat4 uMVP;
        uniform mat4 uBones[128];
        out vec3 vNormal;
        out vec3 vWorldPos;

        void main() {
            mat4 skin = mat4(0.0);
            for (int i = 0; i < 4; ++i)
                if (aBoneIdx[i] >= 0)
                    skin += aBoneWeight[i] * uBones[aBoneIdx[i]];
            if (skin == mat4(0.0))
                skin = mat4(1.0);

            vec4 skinnedPos = skin * vec4(aPos, 1.0);
            vec4 skinnedNormal = skin * vec4(aNormal, 0.0);
            gl_Position = uMVP * skinnedPos;
            vNormal = normalize(skinnedNormal.xyz);
            vWorldPos = skinnedPos.xyz;
        }
    )";
    const char* fragSkin = R"(#version 330 core
        in vec3 vNormal;
        in vec3 vWorldPos;
        uniform vec3 uTint = vec3(1.0);
        out vec4 fragColor;

        void main() {
            vec3 L = normalize(vec3(0.5, 1.0, 0.3));
            float diff = max(dot(normalize(vNormal), L), 0.0);
            vec3 ambient = vec3(0.2, 0.2, 0.25);
            vec3 col = uTint * (ambient + diff * vec3(0.8, 0.75, 0.7));
            fragColor = vec4(col, 1.0);
        }
    )";
    skinnedShader_ = linkProgram(vertSkin, fragSkin);

    // ── Line shader ─────────────────────────────────────────────────────
    const char* vertLine = R"(#version 330 core
        layout(location = 0) in vec3 aPos;
        uniform mat4 uMVP;
        void main() { gl_Position = uMVP * vec4(aPos, 1.0); }
    )";
    const char* fragLine = R"(#version 330 core
        uniform vec3 uColor;
        out vec4 fragColor;
        void main() { fragColor = vec4(uColor, 1.0); }
    )";
    lineShader_ = linkProgram(vertLine, fragLine);

    // ── Upload model geometry to GPU ────────────────────────────────────
    glGenVertexArrays(1, &model_.vao);
    glGenBuffers(1, &model_.vbo);
    glGenBuffers(1, &model_.ebo);

    glBindVertexArray(model_.vao);

    glBindBuffer(GL_ARRAY_BUFFER, model_.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 model_.vertices.size() * sizeof(SkinnedVertex),
                 model_.vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, model_.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 model_.indices.size() * sizeof(unsigned int),
                 model_.indices.data(), GL_STATIC_DRAW);

    // Position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                          sizeof(SkinnedVertex), (void*)0);
    glEnableVertexAttribArray(0);
    // Normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE,
                          sizeof(SkinnedVertex), (void*)sizeof(Vec3));
    glEnableVertexAttribArray(1);
    // Bone indices (integer)
    glVertexAttribIPointer(2, 4, GL_INT, sizeof(SkinnedVertex),
                           (void*)(sizeof(Vec3) * 2));
    glEnableVertexAttribArray(2);
    // Bone weights
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(SkinnedVertex),
                          (void*)(sizeof(Vec3) * 2 + 4 * sizeof(int)));
    glEnableVertexAttribArray(3);

    glBindVertexArray(0);

    // Line VAO (empty, filled at draw time)
    glGenVertexArrays(1, &lineVao_);
    glGenBuffers(1, &lineVbo_);

    glEnable(GL_DEPTH_TEST);
}

// ── Forward Kinematics (sensor quat → bone world matrices) ──────────────
void Renderer::computeFK(const std::array<Quat, 10>& pose,
                         std::vector<Mat4>& out) {
    out.resize(model_.bones().size());
    // Map sensor to bone index (using name matching)
    static int sensorToBone[10] = {-1};
    static bool mapBuilt = false;
    if (!mapBuilt) {
        mapBuilt = true;
        for (int s = 0; s < 10; ++s) {
            sensorToBone[s] = -1;
            for (int b = 0; b < (int)model_.bones().size(); ++b) {
                if (model_.bones()[b].name == kSkeletonBones[s].boneName) {
                    sensorToBone[s] = b;
                    break;
                }
            }
        }
    }

    // Local rotations (identity for non‑driven bones)
    for (auto& m : out) m = Mat4();  // identity
    for (int s = 0; s < 10; ++s) {
        int boneIdx = sensorToBone[s];
        if (boneIdx < 0) continue;
        const Quat& q = pose[s];
        float x = q.x, y = q.y, z = q.z, w = q.w;
        Mat4 rot;
        rot.m[0] = 1 - 2*(y*y + z*z);  rot.m[4] = 2*(x*y - w*z);  rot.m[8]  = 2*(x*z + w*y);
        rot.m[1] = 2*(x*y + w*z);      rot.m[5] = 1 - 2*(x*x + z*z); rot.m[9]  = 2*(y*z - w*x);
        rot.m[2] = 2*(x*z - w*y);      rot.m[6] = 2*(y*z + w*x);      rot.m[10] = 1 - 2*(x*x + y*y);
        rot.m[15] = 1.0f;
        out[boneIdx] = rot;
    }

    // Multiply parent transforms (FK)
    for (size_t i = 1; i < out.size(); ++i) {
        int p = model_.bones()[i].parentBoneIndex;
        if (p >= 0) {
            // out[i] = out[p] * out[i]
            Mat4 a = out[p];
            Mat4 b = out[i];
            Mat4 res;
            for (int c = 0; c < 4; ++c) {
                for (int r = 0; r < 4; ++r) {
                    res.m[c*4 + r] = a.m[0*4 + r] * b.m[c*4 + 0]
                                   + a.m[1*4 + r] * b.m[c*4 + 1]
                                   + a.m[2*4 + r] * b.m[c*4 + 2]
                                   + a.m[3*4 + r] * b.m[c*4 + 3];
                }
            }
            out[i] = res;
        }
    }
}

// ── MVP for a single skeleton (placed at offset) ────────────────────────
Mat4 Renderer::computeMVP(const Vec3& offset, float scale) {
    Vec3 eye    = {offset.x, offset.y + scale * 0.2f, offset.z + scale * 2.8f};
    Vec3 target = {offset.x, offset.y, offset.z};
    Mat4 view = Mat4::lookAt(eye, target, {0, 1, 0});
    Mat4 proj = Mat4::perspective(50.0f * 3.14159f / 180.0f,
                                  1280.0f / 720.0f,
                                  0.1f, scale * 10.0f);
    // Multiply proj * view
    Mat4 mvp;
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) {
            mvp.m[c*4 + r] = proj.m[0*4 + r] * view.m[c*4 + 0]
                           + proj.m[1*4 + r] * view.m[c*4 + 1]
                           + proj.m[2*4 + r] * view.m[c*4 + 2]
                           + proj.m[3*4 + r] * view.m[c*4 + 3];
        }
    return mvp;
}

// ── Draw one skeleton (mesh + lines) ────────────────────────────────────
void Renderer::drawSkeleton(const Mat4& vp, const Vec3& tint,
                            const Vec3& lineCol) {
    // Skinned mesh
    glUseProgram(skinnedShader_);
    glUniformMatrix4fv(glGetUniformLocation(skinnedShader_, "uMVP"),
                       1, GL_FALSE, vp.m);
    glUniform3f(glGetUniformLocation(skinnedShader_, "uTint"),
                tint.x, tint.y, tint.z);
    for (size_t i = 0; i < boneMatrices_.size(); ++i) {
        char name[64];
        snprintf(name, sizeof(name), "uBones[%zu]", i);
        GLint loc = glGetUniformLocation(skinnedShader_, name);
        if (loc >= 0)
            glUniformMatrix4fv(loc, 1, GL_FALSE, boneMatrices_[i].m);
    }
    model_.draw();

    // Skeleton lines
    const auto& bones = model_.bones();
    std::vector<float> lineVerts;
    for (size_t i = 0; i < bones.size(); ++i) {
        int p = bones[i].parentBoneIndex;
        if (p >= 0) {
            // Parent joint position
            lineVerts.push_back(boneMatrices_[p].m[12]);
            lineVerts.push_back(boneMatrices_[p].m[13]);
            lineVerts.push_back(boneMatrices_[p].m[14]);
            // Child joint position
            lineVerts.push_back(boneMatrices_[i].m[12]);
            lineVerts.push_back(boneMatrices_[i].m[13]);
            lineVerts.push_back(boneMatrices_[i].m[14]);
        }
    }
    glUseProgram(lineShader_);
    glUniformMatrix4fv(glGetUniformLocation(lineShader_, "uMVP"),
                       1, GL_FALSE, vp.m);
    glUniform3f(glGetUniformLocation(lineShader_, "uColor"),
                lineCol.x, lineCol.y, lineCol.z);
    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER, lineVerts.size() * sizeof(float),
                 lineVerts.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_LINES, 0, lineVerts.size() / 3);
}

// ── Live mode (single model) ────────────────────────────────────────────
void Renderer::renderLive(const std::array<Quat, 10>& pose) {
    glClearColor(0.15f, 0.15f, 0.17f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    computeFK(pose, boneMatrices_);

    float scale = 0.01f;  // adjust to your model's units (cm → m)
    Vec3 origin = {0.0f, 0.8f * scale, 0.0f};
    Mat4 vp = computeMVP(origin, scale);

    drawSkeleton(vp, {1,1,1}, {0,1,1});   // white mesh, cyan lines

    glfwSwapBuffers(window_);
    glfwPollEvents();
}

// ── Comparison mode (teacher left, student right) ───────────────────────
void Renderer::renderCompare(const std::array<Quat, 10>& teacherPose,
                             const std::array<Quat, 10>& studentPose) {
    glClearColor(0.15f, 0.15f, 0.17f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float scale = 0.01f;
    float separation = 0.6f * scale;

    // Teacher (left, blue tint, cyan lines)
    Vec3 leftOffset  = {-separation, 0.8f * scale, 0.0f};
    computeFK(teacherPose, boneMatrices_);
    Mat4 vpL = computeMVP(leftOffset, scale);
    drawSkeleton(vpL, {0.2f, 0.4f, 1.0f}, {0,1,1});

    // Student (right, red tint, red lines)
    Vec3 rightOffset = { separation, 0.8f * scale, 0.0f};
    computeFK(studentPose, boneMatrices_);
    Mat4 vpR = computeMVP(rightOffset, scale);
    drawSkeleton(vpR, {1.0f, 0.2f, 0.2f}, {1,0,0});

    glfwSwapBuffers(window_);
    glfwPollEvents();
}

} // namespace mocap