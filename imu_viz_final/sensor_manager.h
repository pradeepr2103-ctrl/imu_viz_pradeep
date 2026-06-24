// sensor_manager.h
#pragma once
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <atomic>
#include <mutex>
#include <chrono>

class SensorManager {
public:
    SensorManager();
    
    void setLFAQuat(const glm::quat& q);
    void setRFAQuat(const glm::quat& q);
    void setLUAQuat(const glm::quat& q);
    void setRUAQuat(const glm::quat& q);
    
    glm::quat getLFAQuat() const;
    glm::quat getRFAQuat() const;
    glm::quat getLUAQuat() const;
    glm::quat getRUAQuat() const;
    
    void calibrateLFA();
    void calibrateRFA();
    void calibrateLUA();
    void calibrateRUA();
    void toggleQuaternionConvention();
    void toggleVerticalOffset();
    void togglePlacementGuideMode();
    int  getQuaternionMode() const;
    bool isPlacementGuideMode() const;
    
    glm::quat getCorrectedLFAQuat() const;
    glm::quat getCorrectedRFAQuat() const;
    glm::quat getCorrectedLUAQuat() const;
    glm::quat getCorrectedRUAQuat() const;

    void setLTHQuat(const glm::quat& q);
    void setLSHQuat(const glm::quat& q);
    void setRTHQuat(const glm::quat& q);
    void setRSHQuat(const glm::quat& q);

    glm::quat getLTHQuat() const;
    glm::quat getLSHQuat() const;
    glm::quat getRTHQuat() const;
    glm::quat getRSHQuat() const;

    void calibrateLTH();
    void calibrateLSH();
    void calibrateRTH();
    void calibrateRSH();

    glm::quat getCorrectedLTHQuat() const;
    glm::quat getCorrectedLSHQuat() const;
    glm::quat getCorrectedRTHQuat() const;
    glm::quat getCorrectedRSHQuat() const;

    void setHipsQuat(const glm::quat& q);
    void setChestQuat(const glm::quat& q);

    glm::quat getHipsQuat()  const;
    glm::quat getChestQuat() const;

    void calibrateHips();
    void calibrateChest();

    glm::quat getCorrectedHipsQuat()  const;
    glm::quat getCorrectedChestQuat() const;

    bool isLFAActive()   const;
    bool isRFAActive()   const;
    bool isLUAActive()   const;
    bool isRUAActive()   const;
    bool isLTHActive()   const;
    bool isLSHActive()   const;
    bool isRTHActive()   const;
    bool isRSHActive()   const;
    bool isHipsActive()  const;
    bool isChestActive() const;

private:
    static constexpr float kSmoothingAlpha      = 0.35f;
    static constexpr float kFastSmoothingAlpha  = 0.70f;
    static constexpr float kDeadbandRadians     = 0.003f;
    static constexpr float kFastMotionRadians   = 0.20f;
    static constexpr float kStationaryThreshold = 0.002f;
    static constexpr float kStationaryTimeMs    = 1500.0f;
    static constexpr float kDriftCorrAlpha      = 0.002f;
    static constexpr int   kSensorTimeoutMs     = 500;

    std::atomic<int>  quaternionConvention;
    std::atomic<bool> verticalMode;
    std::atomic<bool> placementGuideMode;

    struct AxisMap {
        int src;
        float sign;
    };
    static constexpr AxisMap verticalRemap[4] = {
        {1,  1.0f},
        {0,  1.0f},
        {2,  1.0f},
        {3,  1.0f}
    };
    
    static glm::quat remapQuat(const glm::quat& q) {
        float comps[4] = {q.x, q.y, q.z, q.w};
        glm::quat result;
        result.x = verticalRemap[0].sign * comps[verticalRemap[0].src];
        result.y = verticalRemap[1].sign * comps[verticalRemap[1].src];
        result.z = verticalRemap[2].sign * comps[verticalRemap[2].src];
        result.w = verticalRemap[3].sign * comps[verticalRemap[3].src];
        return glm::normalize(result);
    }

    // ── sensor state ───────────────────────────────────────────────────────
    // LFA
    mutable std::mutex quatMutex1;
    glm::quat sensorQuatLFA;
    bool activeLFA = false;
    std::chrono::steady_clock::time_point lastReceivedLFA;
    mutable std::mutex calibMutex1;
    mutable glm::quat calibrationReferenceLFA;
    mutable glm::quat smoothedCorrectedLFA;
    mutable bool hasSmoothedCorrectedLFA;
    mutable glm::quat lastQLFA;
    mutable float stationaryTimerLFA;
    mutable glm::quat smoothedLocalLFA;
    mutable bool hasSmoothedLocalLFA;

    // RFA
    mutable std::mutex quatMutex2;
    glm::quat sensorQuatRFA;
    bool activeRFA = false;
    std::chrono::steady_clock::time_point lastReceivedRFA;
    mutable std::mutex calibMutex2;
    mutable glm::quat calibrationReferenceRFA;
    mutable glm::quat smoothedCorrectedRFA;
    mutable bool hasSmoothedCorrectedRFA;
    mutable glm::quat lastQRFA;
    mutable float stationaryTimerRFA;
    mutable glm::quat smoothedLocalRFA;
    mutable bool hasSmoothedLocalRFA;

    // LUA
    mutable std::mutex quatMutex3;
    glm::quat sensorQuat3;
    bool activeLUA = false;
    std::chrono::steady_clock::time_point lastReceivedLUA;
    mutable std::mutex calibMutex3;
    mutable glm::quat calibrationReference3;
    mutable glm::quat smoothedCorrected3;
    mutable bool hasSmoothedCorrected3;
    mutable glm::quat lastQ3;
    mutable float stationaryTimer3;
    mutable glm::quat smoothedLocalLUA;       // NEW
    mutable bool hasSmoothedLocalLUA;         // NEW

    // RUA
    mutable std::mutex quatMutex4;
    glm::quat sensorQuatRUA;
    bool activeRUA = false;
    std::chrono::steady_clock::time_point lastReceivedRUA;
    mutable std::mutex calibMutex4;
    mutable glm::quat calibrationReferenceRUA;
    mutable glm::quat smoothedCorrectedRUA;
    mutable bool hasSmoothedCorrectedRUA;
    mutable glm::quat lastQRUA;
    mutable float stationaryTimerRUA;
    mutable glm::quat smoothedLocalRUA;       // NEW
    mutable bool hasSmoothedLocalRUA;         // NEW

    // LTH
    mutable std::mutex quatMutex5;
    glm::quat sensorQuat5;
    bool activeLTH = false;
    std::chrono::steady_clock::time_point lastReceivedLTH;
    mutable std::mutex calibMutex5;
    mutable glm::quat calibrationReference5;
    mutable glm::quat smoothedCorrected5;
    mutable bool hasSmoothedCorrected5;
    mutable glm::quat lastQ5;
    mutable float stationaryTimer5;
    mutable glm::quat smoothedLocalLTH;       // NEW
    mutable bool hasSmoothedLocalLTH;         // NEW

    // LSH
    mutable std::mutex quatMutex6;
    glm::quat sensorQuat6;
    bool activeLSH = false;
    std::chrono::steady_clock::time_point lastReceivedLSH;
    mutable std::mutex calibMutex6;
    mutable glm::quat calibrationReference6;
    mutable glm::quat smoothedCorrected6;
    mutable bool hasSmoothedCorrected6;
    mutable glm::quat lastQ6;
    mutable float stationaryTimer6;
    mutable glm::quat smoothedLocalLSH;
    mutable bool hasSmoothedLocalLSH;

    // RTH
    mutable std::mutex quatMutex7;
    glm::quat sensorQuat7;
    bool activeRTH = false;
    std::chrono::steady_clock::time_point lastReceivedRTH;
    mutable std::mutex calibMutex7;
    mutable glm::quat calibrationReference7;
    mutable glm::quat smoothedCorrected7;
    mutable bool hasSmoothedCorrected7;
    mutable glm::quat lastQ7;
    mutable float stationaryTimer7;
    mutable glm::quat smoothedLocalRTH;       // NEW
    mutable bool hasSmoothedLocalRTH;         // NEW

    // RSH
    mutable std::mutex quatMutex8;
    glm::quat sensorQuat8;
    bool activeRSH = false;
    std::chrono::steady_clock::time_point lastReceivedRSH;
    mutable std::mutex calibMutex8;
    mutable glm::quat calibrationReference8;
    mutable glm::quat smoothedCorrected8;
    mutable bool hasSmoothedCorrected8;
    mutable glm::quat lastQ8;
    mutable float stationaryTimer8;
    mutable glm::quat smoothedLocalRSH;
    mutable bool hasSmoothedLocalRSH;

    // HIPS
    mutable std::mutex quatMutex9;
    glm::quat sensorQuat9;
    bool activeHips = false;
    std::chrono::steady_clock::time_point lastReceivedHips;
    mutable std::mutex calibMutex9;
    mutable glm::quat calibrationReference9;
    mutable glm::quat smoothedCorrected9;
    mutable bool hasSmoothedCorrected9;
    mutable glm::quat lastQ9;
    mutable float stationaryTimer9;

    // CHEST
    mutable std::mutex quatMutex10;
    glm::quat sensorQuat10;
    bool activeChest = false;
    std::chrono::steady_clock::time_point lastReceivedChest;
    mutable std::mutex calibMutex10;
    mutable glm::quat calibrationReference10;
    mutable glm::quat smoothedCorrected10;
    mutable bool hasSmoothedCorrected10;
    mutable glm::quat lastQ10;
    mutable float stationaryTimer10;

    void updateSensorQuat(glm::quat& current, const glm::quat& incoming) const;
    glm::quat neutralPose() const;
    glm::quat computeMotionDelta(const glm::quat& sensorQuat,
                                 const glm::quat& calibrationReference,
                                 bool applyVertical) const;
    glm::quat computeCorrectedQuat(const glm::quat& sensorQuat,
                                   const glm::quat& calibrationReference) const;
    glm::quat smoothCorrectedQuat(glm::quat& current,
                                  bool& initialized,
                                  const glm::quat& target) const;
    void autoRecalibrate(glm::quat& calibRef,
                         glm::quat& lastQ,
                         float& stationaryTimer,
                         const glm::quat& current) const;
};