#pragma once
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include "gltf_model.h"

enum class CameraView { FRONT, BACK, SIDE };

class Renderer {
public:
    void initialize();
    void render(const glm::quat& leftForearmQ,  const glm::quat& rightForearmQ,
                const glm::quat& leftUpperArmQ, const glm::quat& rightUpperArmQ,
                const glm::quat& leftThighQ,    const glm::quat& rightThighQ,
                const glm::quat& leftShinQ,     const glm::quat& rightShinQ,
                const glm::quat& hipsQ,         const glm::quat& chestQ,
                bool placementGuideMode);
    void setCameraView(CameraView view) { cameraView = view; }
    void cycleCameraView();
    void resetTorsoNeutral() { humanModel.resetTorsoNeutral(); }

private:
    CameraView cameraView = CameraView::FRONT;

    void setupLighting();
    void drawWorldAxes();
    void drawTrackingAxesHud(const glm::quat& lFAQ,
                             const glm::quat& rFAQ,
                             const glm::quat& lUAQ,
                             const glm::quat& rUAQ,
                             const glm::quat& lTHQ,
                             const glm::quat& rTHQ,
                             const glm::quat& lSHQ,
                             const glm::quat& rSHQ,
                             const glm::quat& hipsQ,
                             const glm::quat& chestQ);
    void drawHudAxisWidget(float cx, float cy,
                           const glm::quat& q,
                           float scale);
    void drawPlacementGuideOverlay();

    const int WINDOW_WIDTH  = 1400;
    const int WINDOW_HEIGHT = 900;

    GltfModel humanModel;
};