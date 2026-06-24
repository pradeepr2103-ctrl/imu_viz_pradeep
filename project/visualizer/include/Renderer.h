#pragma once
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <array>
#include <vector>
#include <string>
#include "Model.h"
#include "MathTypes.h"

namespace mocap {

struct RenderState {
    bool  isCompareMode = false;
    float cameraDist    = 2.5f;
    float cameraAngleY  = 0.f;
};

class Renderer {
public:
    explicit Renderer(const std::string& glbPath);
    ~Renderer();

    bool        shouldClose() const { return glfwWindowShouldClose(window_); }
    GLFWwindow* window()      const { return window_; }

    void renderLive(const std::array<Quat,kNumSensors>& pose);
    void renderCompare(const std::array<Quat,kNumSensors>& teacherPose,
                       const std::array<Quat,kNumSensors>& studentPose);
    void drawHUD(bool recording, bool playing, bool isCompare,
                 double playbackTime, const std::string& status);

private:
    GLFWwindow* window_ = nullptr;
    Model       model_;

    unsigned skinnedProg_= 0;

    int uMVP_=-1, uView_=-1, uModel_=-1;
    int uBones_=-1;
    int uTint_=-1, uLightDir_=-1;
    int uAmbient_=-1, uDiffuse_=-1, uSpec_=-1;

    std::vector<Mat4> boneWorld_;
    std::vector<Mat4> skinMat_;

    int  sensorToBone_[kNumSensors];
    // Bind-pose world-space quaternion for each sensor bone.
    // Used to convert absolute IMU quaternions to parent-relative local quats.
    Quat bindWorldQuat_[kNumSensors];

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
