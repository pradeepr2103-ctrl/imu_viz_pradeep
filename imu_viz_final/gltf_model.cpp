#define CGLTF_IMPLEMENTATION
#include "third_party/cgltf.h"

#include "gltf_model.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

#include <GL/glut.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

static glm::quat invertYaw(const glm::quat& q);
static glm::quat yawOnly(const glm::quat& q);

namespace {
glm::mat4 readMat4(const cgltf_accessor* accessor, cgltf_size index)
{
    cgltf_float values[16] = {};
    cgltf_accessor_read_float(accessor, index, values, 16);
    glm::mat4 result(1.0f);
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            result[c][r] = values[c * 4 + r];
        }
    }
    return result;
}

glm::mat4 nodeLocalFromCgltf(const cgltf_node& node)
{
    if (node.has_matrix) {
        glm::mat4 matrix(1.0f);
        for (int c = 0; c < 4; ++c) {
            for (int r = 0; r < 4; ++r) {
                matrix[c][r] = node.matrix[c * 4 + r];
            }
        }
        return matrix;
    }

    glm::vec3 t(0.0f);
    glm::quat r(1.0f, 0.0f, 0.0f, 0.0f);
    glm::vec3 s(1.0f);

    if (node.has_translation) {
        t = glm::vec3(node.translation[0], node.translation[1], node.translation[2]);
    }
    if (node.has_rotation) {
        r = glm::normalize(glm::quat(node.rotation[3], node.rotation[0], node.rotation[1], node.rotation[2]));
    }
    if (node.has_scale) {
        s = glm::vec3(node.scale[0], node.scale[1], node.scale[2]);
    }

    return glm::translate(glm::mat4(1.0f), t) * glm::toMat4(r) * glm::scale(glm::mat4(1.0f), s);
}

glm::vec4 materialColor(const cgltf_material* material)
{
    if (!material) {
        return glm::vec4(0.82f, 0.72f, 0.62f, 1.0f);
    }
    if (material->has_pbr_metallic_roughness) {
        const cgltf_float* c = material->pbr_metallic_roughness.base_color_factor;
        return glm::vec4(c[0], c[1], c[2], c[3]);
    }
    return glm::vec4(0.82f, 0.72f, 0.62f, 1.0f);
}
}

bool GltfModel::load(const std::string& path)
{
    cgltf_options options = {};
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse_file(&options, path.c_str(), &data);
    if (result != cgltf_result_success) {
        std::cerr << "Failed to parse GLB: " << path << "\n";
        return false;
    }

    result = cgltf_load_buffers(&options, data, path.c_str());
    if (result != cgltf_result_success) {
        std::cerr << "Failed to load GLB buffers: " << path << "\n";
        cgltf_free(data);
        return false;
    }

    nodes.clear();
    skins.clear();
    meshes.clear();
    rootNodes.clear();

    std::unordered_map<const cgltf_node*, int> nodeMap;
    nodes.resize(data->nodes_count);
    for (cgltf_size i = 0; i < data->nodes_count; ++i) {
        const cgltf_node& src = data->nodes[i];
        Node& dst = nodes[i];
        dst.name = src.name ? src.name : "";
        dst.localMatrix = nodeLocalFromCgltf(src);
        dst.hasMatrix = src.has_matrix;
        if (src.has_translation) {
            dst.translation = glm::vec3(src.translation[0], src.translation[1], src.translation[2]);
        }
        if (src.has_rotation) {
            dst.rotation = glm::normalize(glm::quat(src.rotation[3], src.rotation[0], src.rotation[1], src.rotation[2]));
        }
        if (src.has_scale) {
            dst.scale = glm::vec3(src.scale[0], src.scale[1], src.scale[2]);
        }

        if (dst.name == "mixamorig:LeftShoulder" || dst.name == "mixamorig:RightShoulder") {
            constexpr float kShoulderWidthScale = 4.0f;
            dst.translation.x *= kShoulderWidthScale;
        }
        nodeMap[&src] = static_cast<int>(i);
    }

    for (cgltf_size i = 0; i < data->nodes_count; ++i) {
        const cgltf_node& src = data->nodes[i];
        Node& dst = nodes[i];
        if (src.parent) {
            dst.parent = nodeMap[src.parent];
        }
        for (cgltf_size c = 0; c < src.children_count; ++c) {
            dst.children.push_back(nodeMap[src.children[c]]);
        }
        if (dst.parent < 0) {
            rootNodes.push_back(static_cast<int>(i));
        }
    }

    skins.resize(data->skins_count);
    for (cgltf_size i = 0; i < data->skins_count; ++i) {
        const cgltf_skin& src = data->skins[i];
        Skin& dst = skins[i];
        dst.joints.resize(src.joints_count);
        dst.inverseBindMatrices.resize(src.joints_count, glm::mat4(1.0f));
        for (cgltf_size j = 0; j < src.joints_count; ++j) {
            dst.joints[j] = nodeMap[src.joints[j]];
            if (src.inverse_bind_matrices) {
                dst.inverseBindMatrices[j] = readMat4(src.inverse_bind_matrices, j);
            }
        }
    }

    for (cgltf_size n = 0; n < data->nodes_count; ++n) {
        const cgltf_node& node = data->nodes[n];
        if (!node.mesh) {
            continue;
        }

        MeshInstance instance;
        instance.nodeIndex = static_cast<int>(n);
        instance.skinIndex = node.skin ? static_cast<int>(node.skin - data->skins) : -1;

        for (cgltf_size p = 0; p < node.mesh->primitives_count; ++p) {
            const cgltf_primitive& srcPrim = node.mesh->primitives[p];
            if (srcPrim.type != cgltf_primitive_type_triangles) {
                continue;
            }

            const cgltf_accessor* pos = nullptr;
            const cgltf_accessor* norm = nullptr;
            const cgltf_accessor* weights = nullptr;
            const cgltf_accessor* joints = nullptr;
            for (cgltf_size a = 0; a < srcPrim.attributes_count; ++a) {
                const cgltf_attribute& attr = srcPrim.attributes[a];
                if (attr.type == cgltf_attribute_type_position) pos = attr.data;
                if (attr.type == cgltf_attribute_type_normal) norm = attr.data;
                if (attr.type == cgltf_attribute_type_weights) weights = attr.data;
                if (attr.type == cgltf_attribute_type_joints) joints = attr.data;
            }
            if (!pos) {
                continue;
            }

            Primitive prim;
            prim.color = materialColor(srcPrim.material);
            prim.vertices.resize(pos->count);
            for (cgltf_size v = 0; v < pos->count; ++v) {
                cgltf_float pv[3] = {};
                cgltf_accessor_read_float(pos, v, pv, 3);
                prim.vertices[v].position = glm::vec3(pv[0], pv[1], pv[2]);

                if (norm) {
                    cgltf_float nv[3] = {};
                    cgltf_accessor_read_float(norm, v, nv, 3);
                    prim.vertices[v].normal = glm::normalize(glm::vec3(nv[0], nv[1], nv[2]));
                }

                if (weights) {
                    cgltf_float wv[4] = {};
                    cgltf_accessor_read_float(weights, v, wv, 4);
                    prim.vertices[v].weights = glm::vec4(wv[0], wv[1], wv[2], wv[3]);
                }

                if (joints) {
                    cgltf_uint jv[4] = {};
                    cgltf_accessor_read_uint(joints, v, jv, 4);
                    for (int k = 0; k < 4; ++k) {
                        prim.vertices[v].joints[k] = static_cast<int>(jv[k]);
                    }
                }
            }

            if (srcPrim.indices) {
                prim.indices.resize(srcPrim.indices->count);
                for (cgltf_size i = 0; i < srcPrim.indices->count; ++i) {
                    prim.indices[i] = static_cast<unsigned int>(cgltf_accessor_read_index(srcPrim.indices, i));
                }
            } else {
                prim.indices.resize(prim.vertices.size());
                for (cgltf_size i = 0; i < prim.vertices.size(); ++i) {
                    prim.indices[i] = static_cast<unsigned int>(i);
                }
            }

            instance.primitives.push_back(std::move(prim));
        }

        if (!instance.primitives.empty()) {
            meshes.push_back(std::move(instance));
        }
    }

    animatedLocalRotations.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        animatedLocalRotations[i] = nodes[i].rotation;
    }

    bindGlobals.resize(nodes.size(), glm::mat4(1.0f));
    animatedGlobals.resize(nodes.size(), glm::mat4(1.0f));
    computeGlobals();
    bindGlobals = animatedGlobals;

    prepareBoneTarget(leftArm, "mixamorig:LeftArm", "mixamorig:LeftForeArm");
    prepareBoneTarget(leftForeArm, "mixamorig:LeftForeArm", "mixamorig:LeftHand");
    prepareBoneTarget(rightArm, "mixamorig:RightArm", "mixamorig:RightForeArm");
    prepareBoneTarget(rightForeArm, "mixamorig:RightForeArm", "mixamorig:RightHand");
    prepareBoneTarget(leftThigh,  "mixamorig:LeftUpLeg",   "mixamorig:LeftLeg");
    prepareBoneTarget(leftShin,   "mixamorig:LeftLeg",     "mixamorig:LeftFoot");
    prepareBoneTarget(rightThigh, "mixamorig:RightUpLeg",  "mixamorig:RightLeg");
    prepareBoneTarget(rightShin,  "mixamorig:RightLeg",    "mixamorig:RightFoot");
    prepareBoneTarget(hips,  "mixamorig:Hips",   "mixamorig:Spine");
    prepareBoneTarget(chest, "mixamorig:Spine2",  "mixamorig:Neck");

    // Bind-pose world rotation for each torso root — see updateTorsoBones().
    if (hips.valid)  hips.alignment  = bindWorldRotation(hips.node);
    if (chest.valid) chest.alignment = bindWorldRotation(chest.node);

    // Optional twist-helper bones present in some Mixamo exports.
    // If absent, findNode returns -1 and the twist code paths are skipped.
    leftForeArmTwist  = findNode("mixamorig:LeftForeArmTwist");
    rightForeArmTwist = findNode("mixamorig:RightForeArmTwist");

    if (leftForeArmTwist  >= 0) std::cout << "Twist helper found: LeftForeArmTwist\n";
    if (rightForeArmTwist >= 0) std::cout << "Twist helper found: RightForeArmTwist\n";

    loaded = !meshes.empty();
    cgltf_free(data);

    if (loaded) {
        std::cout << "Loaded animated GLB: " << path << "\n";
    }
    return loaded;
}

glm::mat4 GltfModel::composeLocal(const Node& node, const glm::quat& localRotation) const
{
    if (node.hasMatrix) {
        return node.localMatrix;
    }
    return glm::translate(glm::mat4(1.0f), node.translation)
         * glm::toMat4(glm::normalize(localRotation))
         * glm::scale(glm::mat4(1.0f), node.scale);
}

void GltfModel::computeGlobals()
{
    for (int root : rootNodes) {
        computeGlobalsRecursive(root, glm::mat4(1.0f));
    }
}

void GltfModel::computeGlobalsRecursive(int nodeIndex, const glm::mat4& parentMatrix)
{
    animatedGlobals[nodeIndex] = parentMatrix * composeLocal(nodes[nodeIndex], animatedLocalRotations[nodeIndex]);
    for (int child : nodes[nodeIndex].children) {
        computeGlobalsRecursive(child, animatedGlobals[nodeIndex]);
    }
}

void GltfModel::draw(const glm::quat& leftForearmLocal,
                     const glm::quat& rightForearmLocal,
                     const glm::quat& leftUpperArmLocal,
                     const glm::quat& rightUpperArmLocal,
                     const glm::quat& leftThighLocal,
                     const glm::quat& rightThighLocal,
                     const glm::quat& leftShinLocal,
                     const glm::quat& rightShinLocal,
                     const glm::quat& chestWorld,
                     const glm::quat& hipsWorld,
                     bool placementGuideMode)
{
    if (!loaded) return;

    for (size_t i = 0; i < nodes.size(); ++i)
        animatedLocalRotations[i] = nodes[i].rotation;
    computeGlobals();

    updateTorsoBones(hipsWorld, chestWorld);

    updateArmBones(leftForearmLocal, rightForearmLocal,
                   leftUpperArmLocal, rightUpperArmLocal,
                   chestWorld);

    updateLegBones(leftThighLocal, rightThighLocal,
                   leftShinLocal, rightShinLocal, yawOnly(hipsWorld));

    glPushMatrix();
    glScalef(9.7f, 9.7f, 9.7f);
    glTranslatef(0.0f, 0.0f, 0.0f);

    for (const MeshInstance& mesh : meshes) {
        const Skin* skin = (mesh.skinIndex >= 0 && mesh.skinIndex < static_cast<int>(skins.size()))
            ? &skins[mesh.skinIndex]
            : nullptr;
        for (const Primitive& primitive : mesh.primitives) {
            drawPrimitive(primitive, skin);
        }
    }

    if (placementGuideMode) {
        updateSensorMarkers();
        drawSensorMarkers();
    }

    glPopMatrix();
}

// ---------------------------------------------------------------------------
// Full-quaternion bone update helpers (Phase 2)
//
// The previous implementation used applyWorldDirection(), which collapses
// the full sensor quaternion into a single direction vector and then finds the
// minimal-arc (swing-only) rotation that points the bone along that vector.
// Axial rotation (pronation/supination, internal/external shoulder rotation,
// leg/shin twist) was therefore always zero — the direction vector carries no
// information about rotation around the bone's own longitudinal axis.
//
// applyWorldRotation() already existed and is correct:
//     q_local = inv(q_W_parent) * q_W_bone
// All we do here is call it instead of applyWorldDirection(), using the full
// world-space quaternion that was already being computed.
//
// Twist bone support: some Mixamo exports split forearm/thigh twist into a
// separate child bone (LeftForeArmTwist, RightForeArmTwist, etc.).  When those
// nodes are found at load time they are driven at kTwistFrac of the parent's
// axial twist each frame.  If the nodes are absent the code is a no-op.
// ---------------------------------------------------------------------------

// Decompose q_W_bone into swing × twist around boneAxis (world space).
// Returns just the twist component as a quaternion.
static glm::quat extractTwist(const glm::quat& q, const glm::vec3& boneAxis)
{
    glm::vec3 ra(q.x, q.y, q.z);          // vector part
    glm::vec3 proj = glm::dot(ra, boneAxis) * boneAxis;
    glm::quat twist = glm::normalize(glm::quat(q.w, proj.x, proj.y, proj.z));
    return twist;
}


void GltfModel::updateArmBones(const glm::quat& leftForearmLocal,
                               const glm::quat& rightForearmLocal,
                               const glm::quat& leftUpperArmLocal,
                               const glm::quat& rightUpperArmLocal,
                               const glm::quat& chestWorldQ)
{
    // Strategy: use applyWorldDirection for swing (proven stable, correct rest pose),
    // then correct the axial (longitudinal) twist using applyWorldRotation.
    //
    // applyWorldDirection(bone, sensorQuat * restDir) gives perfect swing for any
    // motion and naturally produces arms-down rest pose with restDir = (0,-1,0).
    // It loses only the axial roll, which we restore in a second pass.
    //
    // Axial correction pass:
    //   After the swing is set, compute what world rotation the bone now has,
    //   then find what the sensor WANTS the world rotation to be
    //   (sensorDelta * bindWorldRot), extract just the axial difference, and
    //   apply it as an additional local rotation around the bone axis.

    computeGlobals();

    // ── Left upper arm ─────────────────────────────────────────────────────────
    glm::quat lUAWorld = glm::normalize(chestWorldQ * leftUpperArmLocal);

    // Step 1: swing — point the bone in the sensor direction, arms hang at rest
    applyWorldDirection(leftArm, lUAWorld * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();

    // Step 2: axial twist correction
    {
        glm::quat swingWorld  = nodeWorldRotation(leftArm.node);
        glm::quat targetWorld = glm::normalize(leftUpperArmLocal * bindWorldRotation(leftArm.node));
        // Twist axis = bone's current longitudinal axis (pointing down after swing)
        glm::vec3 boneAxis    = glm::normalize(swingWorld * glm::vec3(0.0f, -1.0f, 0.0f));
        glm::quat axisDiff    = glm::normalize(glm::inverse(swingWorld) * targetWorld);
        glm::quat twistOnly   = extractTwist(axisDiff, glm::vec3(0.0f, -1.0f, 0.0f));
        animatedLocalRotations[leftArm.node] =
            glm::normalize(animatedLocalRotations[leftArm.node] * twistOnly);
        (void)boneAxis;
    }
    computeGlobals();

    // ── Right upper arm ────────────────────────────────────────────────────────
    glm::quat rUAWorld = glm::normalize(chestWorldQ * rightUpperArmLocal);

    applyWorldDirection(rightArm, rUAWorld * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();

    {
        glm::quat swingWorld  = nodeWorldRotation(rightArm.node);
        glm::quat targetWorld = glm::normalize(rightUpperArmLocal * bindWorldRotation(rightArm.node));
        glm::quat axisDiff    = glm::normalize(glm::inverse(swingWorld) * targetWorld);
        glm::quat twistOnly   = extractTwist(axisDiff, glm::vec3(0.0f, -1.0f, 0.0f));
        animatedLocalRotations[rightArm.node] =
            glm::normalize(animatedLocalRotations[rightArm.node] * twistOnly);
    }
    computeGlobals();

    // ── Left forearm ───────────────────────────────────────────────────────────
    glm::quat lFAWorld = glm::normalize(lUAWorld * leftForearmLocal);

    applyWorldDirection(leftForeArm, lFAWorld * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();

    {
        glm::quat swingWorld  = nodeWorldRotation(leftForeArm.node);
        glm::quat targetWorld = glm::normalize(leftForearmLocal * bindWorldRotation(leftForeArm.node));
        glm::quat axisDiff    = glm::normalize(glm::inverse(swingWorld) * targetWorld);
        glm::quat twistOnly   = extractTwist(axisDiff, glm::vec3(0.0f, -1.0f, 0.0f));
        animatedLocalRotations[leftForeArm.node] =
            glm::normalize(animatedLocalRotations[leftForeArm.node] * twistOnly);
    }
    computeGlobals();

    // Optional forearm twist helper bone
    if (leftForeArmTwist >= 0) {
        glm::quat lFACurrent = nodeWorldRotation(leftForeArm.node);
        glm::vec3 faBoneAxis = glm::normalize(lFACurrent * glm::vec3(0.0f, -1.0f, 0.0f));
        glm::quat twist      = extractTwist(lFACurrent, faBoneAxis);
        glm::quat twistLocal = glm::normalize(glm::inverse(nodeWorldRotation(leftArm.node)) * twist);
        float halfAngle = std::acos(glm::clamp(twistLocal.w, -1.0f, 1.0f));
        glm::vec3 axis(twistLocal.x, twistLocal.y, twistLocal.z);
        float axisLen = glm::length(axis);
        if (axisLen > 1e-6f)
            animatedLocalRotations[leftForeArmTwist] =
                glm::angleAxis(halfAngle * 2.0f * kTwistFrac, axis / axisLen);
        computeGlobals();
    }

    // ── Right forearm ──────────────────────────────────────────────────────────
    glm::quat rFAWorld = glm::normalize(rUAWorld * rightForearmLocal);

    applyWorldDirection(rightForeArm, rFAWorld * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();

    {
        glm::quat swingWorld  = nodeWorldRotation(rightForeArm.node);
        glm::quat targetWorld = glm::normalize(rightForearmLocal * bindWorldRotation(rightForeArm.node));
        glm::quat axisDiff    = glm::normalize(glm::inverse(swingWorld) * targetWorld);
        glm::quat twistOnly   = extractTwist(axisDiff, glm::vec3(0.0f, -1.0f, 0.0f));
        animatedLocalRotations[rightForeArm.node] =
            glm::normalize(animatedLocalRotations[rightForeArm.node] * twistOnly);
    }
    computeGlobals();

    if (rightForeArmTwist >= 0) {
        glm::quat rFACurrent = nodeWorldRotation(rightForeArm.node);
        glm::vec3 faBoneAxis = glm::normalize(rFACurrent * glm::vec3(0.0f, -1.0f, 0.0f));
        glm::quat twist      = extractTwist(rFACurrent, faBoneAxis);
        glm::quat twistLocal = glm::normalize(glm::inverse(nodeWorldRotation(rightArm.node)) * twist);
        float halfAngle = std::acos(glm::clamp(twistLocal.w, -1.0f, 1.0f));
        glm::vec3 axis(twistLocal.x, twistLocal.y, twistLocal.z);
        float axisLen = glm::length(axis);
        if (axisLen > 1e-6f)
            animatedLocalRotations[rightForeArmTwist] =
                glm::angleAxis(halfAngle * 2.0f * kTwistFrac, axis / axisLen);
        computeGlobals();
    }
}

void GltfModel::updateLegBones(const glm::quat& leftThighLocal,
                                const glm::quat& rightThighLocal,
                                const glm::quat& leftShinLocal,
                                const glm::quat& rightShinLocal,
                                const glm::quat& hipsWorld)
{
    // Same two-pass strategy as arms: swing via applyWorldDirection (stable rest
    // pose, correct limb direction), then axial twist correction pass.
    computeGlobals();

    // ── Left thigh ─────────────────────────────────────────────────────────────
    glm::quat lTHWorld = glm::normalize(hipsWorld * leftThighLocal);

    applyWorldDirection(leftThigh, lTHWorld * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();

    {
        glm::quat swingWorld  = nodeWorldRotation(leftThigh.node);
        glm::quat targetWorld = glm::normalize(leftThighLocal * bindWorldRotation(leftThigh.node));
        glm::quat axisDiff    = glm::normalize(glm::inverse(swingWorld) * targetWorld);
        glm::quat twistOnly   = extractTwist(axisDiff, glm::vec3(0.0f, -1.0f, 0.0f));
        animatedLocalRotations[leftThigh.node] =
            glm::normalize(animatedLocalRotations[leftThigh.node] * twistOnly);
    }
    computeGlobals();

    // ── Right thigh ────────────────────────────────────────────────────────────
    glm::quat rTHWorld = glm::normalize(hipsWorld * rightThighLocal);

    applyWorldDirection(rightThigh, rTHWorld * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();

    {
        glm::quat swingWorld  = nodeWorldRotation(rightThigh.node);
        glm::quat targetWorld = glm::normalize(rightThighLocal * bindWorldRotation(rightThigh.node));
        glm::quat axisDiff    = glm::normalize(glm::inverse(swingWorld) * targetWorld);
        glm::quat twistOnly   = extractTwist(axisDiff, glm::vec3(0.0f, -1.0f, 0.0f));
        animatedLocalRotations[rightThigh.node] =
            glm::normalize(animatedLocalRotations[rightThigh.node] * twistOnly);
    }
    computeGlobals();

    // ── Left shin ──────────────────────────────────────────────────────────────
    glm::quat lSHWorld = glm::normalize(lTHWorld * leftShinLocal);

    applyWorldDirection(leftShin, lSHWorld * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();

    {
        glm::quat swingWorld  = nodeWorldRotation(leftShin.node);
        glm::quat targetWorld = glm::normalize(leftShinLocal * bindWorldRotation(leftShin.node));
        glm::quat axisDiff    = glm::normalize(glm::inverse(swingWorld) * targetWorld);
        glm::quat twistOnly   = extractTwist(axisDiff, glm::vec3(0.0f, -1.0f, 0.0f));
        animatedLocalRotations[leftShin.node] =
            glm::normalize(animatedLocalRotations[leftShin.node] * twistOnly);
    }
    computeGlobals();

    // ── Right shin ─────────────────────────────────────────────────────────────
    glm::quat rSHWorld = glm::normalize(rTHWorld * rightShinLocal);

    applyWorldDirection(rightShin, rSHWorld * glm::vec3(0.0f, -1.0f, 0.0f));
    computeGlobals();

    {
        glm::quat swingWorld  = nodeWorldRotation(rightShin.node);
        glm::quat targetWorld = glm::normalize(rightShinLocal * bindWorldRotation(rightShin.node));
        glm::quat axisDiff    = glm::normalize(glm::inverse(swingWorld) * targetWorld);
        glm::quat twistOnly   = extractTwist(axisDiff, glm::vec3(0.0f, -1.0f, 0.0f));
        animatedLocalRotations[rightShin.node] =
            glm::normalize(animatedLocalRotations[rightShin.node] * twistOnly);
    }
    computeGlobals();
}

static glm::quat invertYaw(const glm::quat& q)
{
    // Sensor AHRS yaw convention is opposite-handed to render world Y-up.
    // Negating the quaternion's y-component flips yaw sense only —
    // pitch and roll (x, z components) are untouched.
    return glm::quat(q.w, -q.x, q.y, q.z);
}

static glm::quat yawOnly(const glm::quat& q)
{
    glm::vec3 axis(q.x, q.y, q.z);
    glm::vec3 proj = glm::dot(axis, glm::vec3(0.0f, 1.0f, 0.0f)) * glm::vec3(0.0f, 1.0f, 0.0f);
    return glm::normalize(glm::quat(q.w, proj.x, proj.y, proj.z));
}

void GltfModel::updateTorsoBones(const glm::quat& hipsWorld, const glm::quat& chestWorld)
{
    // Full quaternion-driven torso.
    //
    // 1) Neutral capture: don't assume the sensor reads identity at rest.
    //    Accel/gyro-only IMU correction zeroes pitch/roll against gravity but
    //    leaves an arbitrary residual yaw at rest — invisible to the old
    //    direction-vector code (pure yaw doesn't move "up"), but fatal once
    //    the full quaternion drives the bone directly. Capture the live
    //    reading once and drive off the delta from that baseline instead of
    //    off the raw quaternion.
    //
    // 2) Yaw handedness: the sensor's AHRS yaw convention is opposite-handed
    //    to render world Y-up, so the raw delta's yaw sense is inverted
    //    before use. Pitch/roll are unaffected by this correction.
    //
    // 3) target.alignment holds each bone's bind-pose world rotation
    //    (cached once in load()). Composing the corrected delta onto it
    //    gives the bone's exact desired world orientation:
    //        B(t) = invertYaw(S(t) * inverse(S(neutral))) * bindWorldRotation(node)
    //    At S(t) == S(neutral) this reduces to bindWorldRotation(node)
    //    exactly, reproducing the rest pose.
    if (!torsoNeutralCaptured) {
        hipsNeutralSensor    = hipsWorld;
        chestNeutralSensor   = chestWorld;
        torsoNeutralCaptured = true;
    }

    glm::quat hipsDelta  = invertYaw(glm::normalize(hipsWorld  * glm::inverse(hipsNeutralSensor)));
    glm::quat chestDelta = invertYaw(glm::normalize(chestWorld * glm::inverse(chestNeutralSensor)));

    if (hips.valid) {
        glm::quat hipsTargetWorld = glm::normalize(hipsDelta * hips.alignment);
        applyWorldRotation(hips, hipsTargetWorld);
        computeGlobals();
    }

    if (chest.valid) {
        glm::quat chestTargetWorld = glm::normalize(chestDelta * chest.alignment);
        applyWorldRotation(chest, chestTargetWorld);
        computeGlobals();
    }
}

// Returns the T-pose world rotation of a node extracted from bindGlobals.
glm::quat GltfModel::bindWorldRotation(int nodeIndex) const
{
    glm::mat3 basis(bindGlobals[nodeIndex]);
    basis[0] = glm::normalize(basis[0]);
    basis[1] = glm::normalize(basis[1]);
    basis[2] = glm::normalize(basis[2]);
    return glm::normalize(glm::quat_cast(basis));
}

void GltfModel::applyWorldRotation(const BoneTarget& target, const glm::quat& worldRotation)
{
    if (!target.valid) return;

    glm::quat parentWorld(1.0f, 0.0f, 0.0f, 0.0f);
    int parent = nodes[target.node].parent;
    if (parent >= 0) {
        parentWorld = nodeWorldRotation(parent);
    }

    animatedLocalRotations[target.node] = glm::normalize(glm::inverse(parentWorld) * worldRotation);
}

void GltfModel::applyWorldDirection(const BoneTarget& target, const glm::vec3& worldDirection)
{
    if (!target.valid || glm::length(worldDirection) < 1e-6f) return;

    glm::quat parentWorld(1.0f, 0.0f, 0.0f, 0.0f);
    int parent = nodes[target.node].parent;
    if (parent >= 0) {
        parentWorld = nodeWorldRotation(parent);
    }

    glm::vec3 childLocal = nodes[target.child].translation;
    if (glm::length(childLocal) < 1e-6f) return;

    glm::vec3 currentDirection = glm::normalize(nodes[target.node].rotation * glm::normalize(childLocal));
    glm::vec3 targetDirection = glm::normalize(glm::inverse(parentWorld) * glm::normalize(worldDirection));
    glm::quat delta = rotationBetween(currentDirection, targetDirection);
    animatedLocalRotations[target.node] = glm::normalize(delta * nodes[target.node].rotation);
}

void GltfModel::prepareBoneTarget(BoneTarget& target, const char* nodeName, const char* childName)
{
    target.node = findNode(nodeName);
    target.child = findNode(childName);
    target.valid = target.node >= 0 && target.child >= 0;
    if (!target.valid) {
        std::cerr << "Missing GLB bone target: " << nodeName << " -> " << childName << "\n";
        return;
    }
    target.alignment = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
}

int GltfModel::findNode(const std::string& name) const
{
    for (size_t i = 0; i < nodes.size(); ++i)
        if (nodes[i].name == name)
            return static_cast<int>(i);
    return -1;
}

glm::vec3 GltfModel::nodeWorldPosition(int nodeIndex) const
{
    glm::vec4 p = bindGlobals[nodeIndex] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    return glm::vec3(p) / p.w;
}

glm::vec3 GltfModel::animatedWorldPosition(int nodeIndex) const
{
    glm::vec4 p = animatedGlobals[nodeIndex] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    return glm::vec3(p) / p.w;
}

void GltfModel::updateSensorMarkers()
{
    sensorMarkers.clear();

    if (!hips.valid || !chest.valid || !leftArm.valid || !rightArm.valid ||
        !leftForeArm.valid || !rightForeArm.valid || !leftThigh.valid ||
        !rightThigh.valid || !leftShin.valid || !rightShin.valid) {
        return;
    }

    glm::vec3 hipPos    = animatedWorldPosition(hips.node);
    glm::vec3 chestPos  = animatedWorldPosition(chest.node);
    glm::vec3 lShoulder = animatedWorldPosition(leftArm.node);
    glm::vec3 rShoulder = animatedWorldPosition(rightArm.node);
    glm::vec3 lElbow    = animatedWorldPosition(leftArm.child);
    glm::vec3 rElbow    = animatedWorldPosition(rightArm.child);
    glm::vec3 lWrist    = animatedWorldPosition(leftForeArm.child);
    glm::vec3 rWrist    = animatedWorldPosition(rightForeArm.child);
    glm::vec3 lHipJ     = animatedWorldPosition(leftThigh.node);
    glm::vec3 rHipJ     = animatedWorldPosition(rightThigh.node);
    glm::vec3 lKnee     = animatedWorldPosition(leftThigh.child);
    glm::vec3 rKnee     = animatedWorldPosition(rightThigh.child);
    glm::vec3 lAnkle    = animatedWorldPosition(leftShin.child);
    glm::vec3 rAnkle    = animatedWorldPosition(rightShin.child);

    glm::vec3 up     = glm::normalize(chestPos - hipPos);
    glm::vec3 across = glm::normalize(rShoulder - lShoulder);
    glm::vec3 front  = glm::normalize(glm::cross(up, across));
    float torsoScale = glm::length(chestPos - hipPos);
    markerRadius          = torsoScale * kMarkerRadiusFrac;
    markerUpDir           = up;
    orientationLineLength = torsoScale * kOrientationLineFrac;

    auto lateralOffset = [&](const glm::vec3& mid, const glm::vec3& refCenter) {
        glm::vec3 raw = mid - refCenter;
        glm::vec3 horiz = raw - up * glm::dot(raw, up);
        float len = glm::length(horiz);
        return len > 1e-5f ? horiz / len : front;
    };

    auto addMidpointMarker = [&](const char* label, const glm::vec3& a, const glm::vec3& b,
                                  const glm::vec3& dir, float frac) {
        glm::vec3 mid = 0.5f * (a + b);
        float boneLen = glm::length(b - a);
        sensorMarkers.push_back({label, mid + dir * boneLen * frac});
    };

    sensorMarkers.push_back({"CHEST", chestPos + front * torsoScale * kChestFrontFrac
                                                - up    * torsoScale * kChestDownFrac});
    sensorMarkers.push_back({"HIPS", hipPos + front * torsoScale * 0.4f
                                                - up    * torsoScale * kHipsDownFrac});;
    /*sensorMarkers.push_back({"L_UA", findArmSurfaceMarker(leftArm.node, leftArm.child, lShoulder, lElbow,
                                       lateralOffset(0.5f * (lShoulder + lElbow), chestPos), kUALateralFrac)});
    sensorMarkers.push_back({"R_UA", findArmSurfaceMarker(rightArm.node, rightArm.child, rShoulder, rElbow,
                                       lateralOffset(0.5f * (rShoulder + rElbow), chestPos), kUALateralFrac)});
    sensorMarkers.push_back({"L_FA", findArmSurfaceMarker(leftForeArm.node, leftForeArm.child, lElbow, lWrist,
                                    -across, kFAFrac)});
    sensorMarkers.push_back({"R_FA", findArmSurfaceMarker(rightForeArm.node, rightForeArm.child, rElbow, rWrist,
                                    across, kFAFrac)});*/
    addMidpointMarker("L_UA", lShoulder, lElbow,  front, 0.25f);
    addMidpointMarker("R_UA", rShoulder, rElbow,  front, 0.25f);
    addMidpointMarker("L_FA", lElbow, lWrist, front, 0.25f);
    addMidpointMarker("R_FA", rElbow, rWrist, front, 0.25f);
    addMidpointMarker("L_TH", lHipJ, lKnee,  front, 0.25f);
    addMidpointMarker("R_TH", rHipJ, rKnee,  front, 0.25f);
    addMidpointMarker("L_SH", lKnee, lAnkle, front, 0.15f);
    addMidpointMarker("R_SH", rKnee, rAnkle, front, 0.15f);
}

void GltfModel::drawSensorMarkers()
{
    glDisable(GL_LIGHTING);

    glDepthMask(GL_FALSE);
    glColor4f(0.25f, 1.0f, 0.35f, 0.30f);
    for (const SensorMarker& marker : sensorMarkers) {
        glPushMatrix();
        glTranslatef(marker.worldPos.x, marker.worldPos.y, marker.worldPos.z);
        glutSolidSphere(markerRadius * 1.8, 12, 12);
        glPopMatrix();
    }
    glDepthMask(GL_TRUE);

    glColor4f(0.45f, 1.0f, 0.5f, 1.0f);
    for (const SensorMarker& marker : sensorMarkers) {
        glPushMatrix();
        glTranslatef(marker.worldPos.x, marker.worldPos.y, marker.worldPos.z);
        glutSolidSphere(markerRadius, 12, 12);
        glPopMatrix();
    }

    glColor3f(0.3f, 1.0f, 0.3f);
    glLineWidth(2.5f);
    glBegin(GL_LINES);
    for (const SensorMarker& marker : sensorMarkers) {
        glm::vec3 tip = marker.worldPos + markerUpDir * orientationLineLength;
        glVertex3f(marker.worldPos.x, marker.worldPos.y, marker.worldPos.z);
        glVertex3f(tip.x, tip.y, tip.z);
    }
    glEnd();
    glLineWidth(1.0f);

    for (const SensorMarker& marker : sensorMarkers) {
        glm::vec3 tip = marker.worldPos + markerUpDir * orientationLineLength;
        glPushMatrix();
        glTranslatef(tip.x, tip.y, tip.z);
        glm::vec3 axis = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), markerUpDir);
        float angle = glm::degrees(std::acos(glm::clamp(markerUpDir.z, -1.0f, 1.0f)));
        if (glm::length(axis) > 1e-5f) {
            glRotatef(angle, axis.x, axis.y, axis.z);
        }
        glutSolidCone(markerRadius * 0.6, markerRadius * 1.4, 8, 1);
        glPopMatrix();
    }

    glColor3f(0.65f, 1.0f, 0.7f);
    for (const SensorMarker& marker : sensorMarkers)
    {
        bool rightSide =
            marker.label.rfind("R_", 0) == 0;

        float xOffset =
            rightSide ?
            -markerRadius * 4.5f :
            markerRadius * 1.6f;

        glRasterPos3f(
            marker.worldPos.x + xOffset,
            marker.worldPos.y + markerRadius * 1.6f,
            marker.worldPos.z);

        for (char c : marker.label)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, c);
    }

    glEnable(GL_LIGHTING);
}

bool GltfModel::findJointSlot(int nodeIndex, int& skinIndexOut, int& jointSlotOut) const
{
    for (size_t s = 0; s < skins.size(); ++s) {
        const Skin& skin = skins[s];
        for (size_t j = 0; j < skin.joints.size(); ++j) {
            if (skin.joints[j] == nodeIndex) {
                skinIndexOut = static_cast<int>(s);
                jointSlotOut = static_cast<int>(j);
                return true;
            }
        }
    }
    return false;
}

glm::vec3 GltfModel::skinnedVertexWorldPosition(const Vertex& vertex, const Skin& skin) const
{
    glm::vec4 skinned(0.0f);
    bool any = false;
    for (int i = 0; i < 4; ++i) {
        float weight = vertex.weights[i];
        if (weight <= 0.0f) continue;
        any = true;
        glm::mat4 m = skinMatrix(skin, vertex.joints[i]);
        skinned += weight * (m * glm::vec4(vertex.position, 1.0f));
    }
    return any ? glm::vec3(skinned) : vertex.position;
}

glm::vec3 GltfModel::findArmSurfaceMarker(int jointNode, int childNode,
                                          const glm::vec3& jointPos, const glm::vec3& childPos,
                                          const glm::vec3& desiredDir, float fallbackFrac) const
{
    int skinIndex = -1, jointSlot = -1;
    float boneLen = glm::length(childPos - jointPos);
    glm::vec3 fallback = 0.5f * (jointPos + childPos) + desiredDir * boneLen * fallbackFrac;

    if (!findJointSlot(jointNode, skinIndex, jointSlot)) return fallback;

    const Skin& skin = skins[skinIndex];
    glm::vec3 axisDir = boneLen > 1e-5f ? (childPos - jointPos) / boneLen : glm::vec3(0.0f, -1.0f, 0.0f);

    glm::vec3 bestPoint = fallback;
    float bestScore = -1e9f;
    bool found = false;

    for (const MeshInstance& mesh : meshes) {
        if (mesh.skinIndex != skinIndex) continue;
        for (const Primitive& primitive : mesh.primitives) {
            for (const Vertex& vertex : primitive.vertices) {
                bool dominant = false;
                for (int i = 0; i < 4; ++i) {
                    if (vertex.weights[i] > 0.5f && vertex.joints[i] == jointSlot) {
                        dominant = true;
                        break;
                    }
                }
                if (!dominant) continue;

                glm::vec3 worldPos = skinnedVertexWorldPosition(vertex, skin);
                float t = glm::dot(worldPos - jointPos, axisDir);
                if (t < boneLen * 0.25f || t > boneLen * 0.75f) continue;

                glm::vec3 axisPoint = jointPos + axisDir * t;
                glm::vec3 perp = worldPos - axisPoint;
                float perpLen = glm::length(perp);
                if (perpLen < 1e-5f) continue;

                float alignment = glm::dot(perp / perpLen, desiredDir);
                float score = alignment * perpLen;
                if (score > bestScore) {
                    bestScore = score;

                    glm::vec3 axisPoint = jointPos + axisDir * t;

                    // Move 20% from skin vertex toward bone center
                    bestPoint = glm::mix(worldPos, axisPoint, 0.20f);

                    found = true;
                }
            }
        }
    }

    return found ? bestPoint : fallback;
}

glm::quat GltfModel::nodeWorldRotation(int nodeIndex) const
{
    glm::mat3 basis(animatedGlobals[nodeIndex]);
    basis[0] = glm::normalize(basis[0]);
    basis[1] = glm::normalize(basis[1]);
    basis[2] = glm::normalize(basis[2]);
    return glm::normalize(glm::quat_cast(basis));
}

glm::quat GltfModel::rotationBetween(glm::vec3 from, glm::vec3 to) const
{
    from = glm::normalize(from);
    to = glm::normalize(to);
    float cosTheta = glm::clamp(glm::dot(from, to), -1.0f, 1.0f);
    if (cosTheta > 0.9999f) return glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (cosTheta < -0.9999f) {
        glm::vec3 axis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), from);
        if (glm::length(axis) < 1e-5f)
            axis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), from);
        return glm::angleAxis(glm::radians(180.0f), glm::normalize(axis));
    }
    glm::vec3 axis = glm::normalize(glm::cross(from, to));
    return glm::angleAxis(std::acos(cosTheta), axis);
}

glm::mat4 GltfModel::skinMatrix(const Skin& skin, int jointSlot) const
{
    if (jointSlot < 0 || jointSlot >= static_cast<int>(skin.joints.size()))
        return glm::mat4(1.0f);
    int nodeIndex = skin.joints[jointSlot];
    return animatedGlobals[nodeIndex] * skin.inverseBindMatrices[jointSlot];
}

void GltfModel::drawPrimitive(const Primitive& primitive, const Skin* skin)
{
    glm::vec4 c = primitive.color;
    glColor4f(c.r, c.g, c.b, c.a);
    glBegin(GL_TRIANGLES);

    for (unsigned int index : primitive.indices) {
        if (index >= primitive.vertices.size()) continue;
        const Vertex& vertex = primitive.vertices[index];
        glm::vec4 skinnedPosition(0.0f);
        glm::vec3 skinnedNormal(0.0f);

        if (skin && glm::dot(vertex.weights, glm::vec4(1.0f)) > 0.0f) {
            for (int i = 0; i < 4; ++i) {
                float weight = vertex.weights[i];
                if (weight <= 0.0f) continue;
                glm::mat4 m = skinMatrix(*skin, vertex.joints[i]);
                skinnedPosition += weight * (m * glm::vec4(vertex.position, 1.0f));
                skinnedNormal += weight * glm::mat3(m) * vertex.normal;
            }
        } else {
            skinnedPosition = glm::vec4(vertex.position, 1.0f);
            skinnedNormal = vertex.normal;
        }

        if (glm::length(skinnedNormal) > 1e-6f)
            skinnedNormal = glm::normalize(skinnedNormal);
        else
            skinnedNormal = glm::vec3(0.0f, 1.0f, 0.0f);

        glNormal3fv(glm::value_ptr(skinnedNormal));
        glVertex3fv(glm::value_ptr(glm::vec3(skinnedPosition)));
    }

    glEnd();
}