#pragma once

#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

struct SensorMarker {
    std::string label;
    glm::vec3 worldPos;
};

class GltfModel{
public:
    bool load(const std::string& path);
    bool isLoaded() const { return loaded; }

    void draw(const glm::quat& leftForearmLocal,
              const glm::quat& rightForearmLocal,
              const glm::quat& leftUpperArmLocal,
              const glm::quat& rightUpperArmLocal,
              const glm::quat& leftThighLocal,
              const glm::quat& rightThighLocal,
              const glm::quat& leftShinLocal,
              const glm::quat& rightShinLocal,
              const glm::quat& chestWorld,
              const glm::quat& hipsWorld,
              bool placementGuideMode = false);
    

    void resetTorsoNeutral() { torsoNeutralCaptured = false; }

private:
    struct Vertex {
        glm::vec3 position = glm::vec3(0.0f);
        glm::vec3 normal = glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec4 weights = glm::vec4(0.0f);
        glm::ivec4 joints = glm::ivec4(0);
    };

    struct Primitive {
        std::vector<Vertex> vertices;
        std::vector<unsigned int> indices;
        glm::vec4 color = glm::vec4(0.82f, 0.72f, 0.62f, 1.0f);
    };

    struct MeshInstance {
        int nodeIndex = -1;
        int skinIndex = -1;
        std::vector<Primitive> primitives;
    };

    struct Node {
        std::string name;
        int parent = -1;
        std::vector<int> children;
        glm::vec3 translation = glm::vec3(0.0f);
        glm::quat rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        glm::vec3 scale = glm::vec3(1.0f);
        glm::mat4 localMatrix = glm::mat4(1.0f);
        bool hasMatrix = false;
    };

    struct Skin {
        std::vector<int> joints;
        std::vector<glm::mat4> inverseBindMatrices;
    };

    struct BoneTarget {
        int node = -1;
        int child = -1;
        glm::quat alignment = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        bool valid = false;
    };

    bool loaded = false;
    std::vector<Node> nodes;
    std::vector<Skin> skins;
    std::vector<MeshInstance> meshes;
    std::vector<int> rootNodes;
    std::vector<glm::mat4> bindGlobals;
    std::vector<glm::mat4> animatedGlobals;
    std::vector<glm::quat> animatedLocalRotations;

    BoneTarget leftArm;
    BoneTarget leftForeArm;
    BoneTarget rightArm;
    BoneTarget rightForeArm;
    BoneTarget leftThigh;
    BoneTarget leftShin;
    BoneTarget rightThigh;
    BoneTarget rightShin;
    BoneTarget hips;
    BoneTarget chest;

    // Optional twist-helper bones (present in some Mixamo exports).
    // -1 if the GLB does not contain them.
    int leftForeArmTwist  = -1;
    int rightForeArmTwist = -1;

    bool torsoNeutralCaptured = false;
    glm::quat hipsNeutralSensor  = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::quat chestNeutralSensor = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    void updateTorsoBones(const glm::quat& hipsWorld, const glm::quat& chestWorld);

    glm::mat4 composeLocal(const Node& node, const glm::quat& localRotation) const;
    void computeGlobals();
    void computeGlobalsRecursive(int nodeIndex, const glm::mat4& parentMatrix);

    void updateArmBones(const glm::quat& leftForearmLocal,
                        const glm::quat& rightForearmLocal,
                        const glm::quat& leftUpperArmLocal,
                        const glm::quat& rightUpperArmLocal,
                        const glm::quat& chestWorldQ);

    void updateLegBones(const glm::quat& leftThighLocal,
                        const glm::quat& rightThighLocal,
                        const glm::quat& leftShinLocal,
                        const glm::quat& rightShinLocal,
                        const glm::quat& hipsWorld);

    glm::quat bindWorldRotation(int nodeIndex) const;
    void applyWorldRotation(const BoneTarget& target, const glm::quat& worldRotation);
    void applyWorldDirection(const BoneTarget& target, const glm::vec3& worldDirection);
    void prepareBoneTarget(BoneTarget& target, const char* nodeName, const char* childName);
    int findNode(const std::string& name) const;
    glm::vec3 nodeWorldPosition(int nodeIndex) const;
    glm::quat nodeWorldRotation(int nodeIndex) const;
    glm::quat rotationBetween(glm::vec3 from, glm::vec3 to) const;
    glm::mat4 skinMatrix(const Skin& skin, int jointSlot) const;
    void drawPrimitive(const Primitive& primitive, const Skin* skin);

    static constexpr float kChestFrontFrac    = 0.55f;
    static constexpr float kChestDownFrac     = 0.12f;
    static constexpr float kHipsFrontFrac     = 0.55f;
    static constexpr float kHipsDownFrac      = 0.10f;
    static constexpr float kUALateralFrac     = 0.45f;
    static constexpr float kFAFrac            = 0.45f;
    static constexpr float kTHFrac            = 0.45f;
    static constexpr float kSHFrac            = 0.45f;
    static constexpr float kMarkerRadiusFrac  = 0.09f;
    static constexpr float kOrientationLineFrac = 0.22f;
    // Fraction of forearm axial twist propagated to the twist-helper bone.
    // 0.5 = half twist (standard anatomical split), 0 = disable.
    static constexpr float kTwistFrac = 0.5f;

    std::vector<SensorMarker> sensorMarkers;
    float markerRadius = 0.08f;
    glm::vec3 markerUpDir = glm::vec3(0.0f, 1.0f, 0.0f);
    float orientationLineLength = 0.05f;

    glm::vec3 animatedWorldPosition(int nodeIndex) const;
    void updateSensorMarkers();
    void drawSensorMarkers();

    bool findJointSlot(int nodeIndex, int& skinIndexOut, int& jointSlotOut) const;
    glm::vec3 skinnedVertexWorldPosition(const Vertex& vertex, const Skin& skin) const;
    glm::vec3 findArmSurfaceMarker(int jointNode, int childNode,
                                   const glm::vec3& jointPos, const glm::vec3& childPos,
                                   const glm::vec3& desiredDir, float fallbackFrac) const;
};  