#pragma once
#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <array>
#include <vector>
#include "Model.h"
#include "MathTypes.h"

namespace mocap {

class Renderer {
public:
    Renderer(const std::string& glbPath);
    ~Renderer();

    bool shouldClose() const { return glfwWindowShouldClose(window_); }
    GLFWwindow* window() const { return window_; }

    // Render single skeleton (live mode)
    void renderLive(const std::array<Quat, 10>& pose);

    // Side‑by‑side comparison (teacher left, student right)
    void renderCompare(const std::array<Quat, 10>& teacherPose,
                       const std::array<Quat, 10>& studentPose);

private:
    GLFWwindow* window_ = nullptr;
    Model model_;

    unsigned int skinnedShader_ = 0;
    unsigned int lineShader_    = 0;
    unsigned int lineVao_ = 0, lineVbo_ = 0;

    std::vector<Mat4> boneMatrices_;

    void setupGL();
    void computeFK(const std::array<Quat, 10>& pose, std::vector<Mat4>& out);
    Mat4 computeMVP(const Vec3& offset, float scale);
    void drawSkeleton(const Mat4& vp, const Vec3& tint, const Vec3& lineCol);
};

} // namespace mocap