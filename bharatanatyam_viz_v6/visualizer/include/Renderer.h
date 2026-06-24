#pragma once
// glad MUST be first
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <array>
#include <vector>
#include <string>
#include "Model.h"
#include "MathTypes.h"
#include "Skeleton.h"

namespace mocap {

class Renderer {
public:
    explicit Renderer(const std::string& glbPath);
    ~Renderer();

    bool        shouldClose() const { return glfwWindowShouldClose(window_); }
    GLFWwindow* window()      const { return window_; }

    void renderLive   (const std::array<Quat,kNumSensors>& pose);
    void renderCompare(const std::array<Quat,kNumSensors>& teacherPose,
                       const std::array<Quat,kNumSensors>& studentPose);
    void drawHUD(bool recording, bool playing, bool isCompare,
                 double playbackTime, const std::string& status);

private:
    GLFWwindow* window_ = nullptr;
    Model       model_;

    // Skinned mesh shader
    GLuint skinnedProg_ = 0;
    GLint  uProj_=-1, uView_=-1, uModel_=-1;
    GLint  uBones_=-1, uTint_=-1, uLightDir_=-1;
    GLint  uAmbient_=-1, uDiffuse_=-1, uSpec_=-1;

    // Flat-colour line shader for sensor axis panels
    GLuint lineProg_   = 0;
    GLint  uLineMVP_   = -1;
    GLint  uLineColor_ = -1;

    // Axis geometry (6 verts: 3 axes × 2 endpoints)
    GLuint axisVAO_ = 0;
    GLuint axisVBO_ = 0;

    std::vector<Mat4> boneWorld_;
    std::vector<Mat4> skinMat_;
    int sensorToBone_[kNumSensors];

    void setupShaders();
    void setupAxisVAO();
    void buildSensorMap();
    void computeFK(const std::array<Quat,kNumSensors>& pose);
    void uploadBones();
    void drawMesh(const Mat4& proj, const Mat4& view,
                  const Mat4& mdl, const Vec3& tint);
    void drawSensorPanels(int winW, int winH,
                          const std::array<Quat,kNumSensors>& pose);
    float autoDistance() const;
    Mat4 buildView(float dist, float angleY) const;
    Mat4 buildProj(float aspect) const;
    Vec3 meshCenter() const;
};

} // namespace mocap
