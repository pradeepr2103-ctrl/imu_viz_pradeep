#pragma once
#define GLFW_INCLUDE_NONE
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <array>
#include <vector>
#include <string>
#include "Model.h"
#include "MathTypes.h"
#include "Skeleton.h"

namespace mocap {

struct RenderState {
    bool  isCompareMode = false;
    float cameraDist    = 3.5f;
    float cameraAngleY  = 0.f;     // orbit angle (radians)
};

class Renderer {
public:
    explicit Renderer(const std::string& glbPath);
    ~Renderer();

    bool        shouldClose() const { return glfwWindowShouldClose(window_); }
    GLFWwindow* window()      const { return window_; }

    // Live: single skinned mesh, identity-ish pose
    void renderLive(const std::array<Quat,kNumSensors>& pose);

    // Compare: teacher (left viewport, blue tint) vs student (right, red tint)
    void renderCompare(const std::array<Quat,kNumSensors>& teacherPose,
                       const std::array<Quat,kNumSensors>& studentPose);

    // HUD text overlay (drawn on top each frame)
    void drawHUD(bool recording, bool playing, bool isCompare,
                 double playbackTime, const std::string& status);

private:
    GLFWwindow* window_ = nullptr;
    Model       model_;

    // Shader programs
    unsigned skinnedProg_= 0;
    unsigned hudProg_    = 0;

    // Uniforms
    int uMVP_=-1, uView_=-1, uModel_=-1;
    int uBones_=-1;
    int uTint_=-1, uLightDir_=-1;
    int uAmbient_=-1, uDiffuse_=-1, uSpec_=-1;

    // Bone matrices
    std::vector<Mat4> boneWorld_;
    std::vector<Mat4> skinMat_;

    // Sensor → bone index
    int sensorToBone_[kNumSensors];

    // HUD VAO/VBO for 2D quad rendering
    unsigned hudVao_=0, hudVbo_=0;

    void setupShaders();
    void buildSensorMap();
    void computeFK(const std::array<Quat,kNumSensors>& pose);
    void uploadBones();
    void drawMesh(const Mat4& proj, const Mat4& view, const Mat4& model,
                  const Vec3& tint);
    Mat4 buildView(float distZ, float angleY) const;
    Mat4 buildProj(float aspect) const;
    Vec3 meshCenter() const;
};

} // namespace mocap
