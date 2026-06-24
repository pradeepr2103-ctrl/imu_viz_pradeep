// sensor_manager.cpp
#include "sensor_manager.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <glm/gtx/quaternion.hpp>

SensorManager::SensorManager()
    : sensorQuatLFA(1,0,0,0), sensorQuatRFA(1,0,0,0)
    , sensorQuat3(1,0,0,0),   sensorQuatRUA(1,0,0,0)
    , sensorQuat5(1,0,0,0),   sensorQuat6(1,0,0,0)
    , sensorQuat7(1,0,0,0),   sensorQuat8(1,0,0,0)
    , sensorQuat9(1,0,0,0),   sensorQuat10(1,0,0,0)
    , quaternionConvention(0)
    , verticalMode(false)
    , placementGuideMode(false)
    , calibrationReferenceLFA(1,0,0,0), smoothedCorrectedLFA(1,0,0,0)
    , hasSmoothedCorrectedLFA(false),   lastQLFA(1,0,0,0), stationaryTimerLFA(0)
    , smoothedLocalLFA(1,0,0,0), hasSmoothedLocalLFA(false)
    , calibrationReferenceRFA(1,0,0,0), smoothedCorrectedRFA(1,0,0,0)
    , hasSmoothedCorrectedRFA(false),   lastQRFA(1,0,0,0), stationaryTimerRFA(0)
    , smoothedLocalRFA(1,0,0,0), hasSmoothedLocalRFA(false)
    , calibrationReference3(1,0,0,0),   smoothedCorrected3(1,0,0,0)
    , hasSmoothedCorrected3(false),     lastQ3(1,0,0,0), stationaryTimer3(0)
    , smoothedLocalLUA(1,0,0,0), hasSmoothedLocalLUA(false)
    , calibrationReferenceRUA(1,0,0,0), smoothedCorrectedRUA(1,0,0,0)
    , hasSmoothedCorrectedRUA(false),   lastQRUA(1,0,0,0), stationaryTimerRUA(0)
    , smoothedLocalRUA(1,0,0,0), hasSmoothedLocalRUA(false)
    , calibrationReference5(1,0,0,0),   smoothedCorrected5(1,0,0,0)
    , hasSmoothedCorrected5(false),     lastQ5(1,0,0,0), stationaryTimer5(0)
    , smoothedLocalLTH(1,0,0,0), hasSmoothedLocalLTH(false)
    , calibrationReference6(1,0,0,0),   smoothedCorrected6(1,0,0,0)
    , hasSmoothedCorrected6(false),     lastQ6(1,0,0,0), stationaryTimer6(0)
    , smoothedLocalLSH(1,0,0,0), hasSmoothedLocalLSH(false)
    , calibrationReference7(1,0,0,0),   smoothedCorrected7(1,0,0,0)
    , hasSmoothedCorrected7(false),     lastQ7(1,0,0,0), stationaryTimer7(0)
    , smoothedLocalRTH(1,0,0,0), hasSmoothedLocalRTH(false)
    , calibrationReference8(1,0,0,0),   smoothedCorrected8(1,0,0,0)
    , hasSmoothedCorrected8(false),     lastQ8(1,0,0,0), stationaryTimer8(0)
    , smoothedLocalRSH(1,0,0,0), hasSmoothedLocalRSH(false)
    , calibrationReference9(1,0,0,0),   smoothedCorrected9(1,0,0,0)
    , hasSmoothedCorrected9(false),     lastQ9(1,0,0,0), stationaryTimer9(0)
    , calibrationReference10(1,0,0,0),  smoothedCorrected10(1,0,0,0)
    , hasSmoothedCorrected10(false),    lastQ10(1,0,0,0), stationaryTimer10(0)
{
    std::cout << "Quaternion mode 1/4 (horizontal). Press M to cycle conventions, P to toggle vertical mount.\n";
}

// ── core helpers ──────────────────────────────────────────────────────────────

void SensorManager::updateSensorQuat(glm::quat& current, const glm::quat& incoming) const
{
    glm::quat t = glm::normalize(incoming);
    if (glm::dot(current, t) < 0.0f) t = -t;
    current = t;
}

glm::quat SensorManager::neutralPose() const
{
    return glm::angleAxis(glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::quat SensorManager::computeMotionDelta(const glm::quat& sensorQuat,
                                             const glm::quat& calibrationReference,
                                             bool applyVertical) const
{
    glm::quat current   = glm::normalize(sensorQuat);
    glm::quat reference = glm::normalize(calibrationReference);

    if (applyVertical) {
        current   = remapQuat(current);
        reference = remapQuat(reference);
    }

    if (glm::dot(reference, current) < 0.0f) current = -current;

    int mode = quaternionConvention.load();
    switch (mode) {
    case 0:  return glm::normalize(current * glm::inverse(reference));
    case 1:  return glm::normalize(glm::inverse(reference) * current);
    case 2:  return glm::normalize(glm::conjugate(current) * glm::inverse(glm::conjugate(reference)));
    default: return glm::normalize(glm::inverse(glm::conjugate(reference)) * glm::conjugate(current));
    }
}

glm::quat SensorManager::computeCorrectedQuat(const glm::quat& sensorQuat,
                                               const glm::quat& calibrationReference) const
{
    glm::quat delta = computeMotionDelta(sensorQuat, calibrationReference, verticalMode.load());
    delta.x = -delta.x;
    delta.y = -delta.y;
    delta.z = -delta.z;
    glm::quat q = glm::normalize(delta * neutralPose());
    return glm::normalize(q);
}

glm::quat SensorManager::smoothCorrectedQuat(glm::quat& current,
                                              bool& initialized,
                                              const glm::quat& target) const
{
    glm::quat nt = glm::normalize(target);
    if (!initialized) { current = nt; initialized = true; return current; }
    if (glm::dot(current, nt) < 0.0f) nt = -nt;

    const float dot   = std::clamp(glm::dot(current, nt), -1.0f, 1.0f);
    const float angle = 2.0f * std::acos(std::abs(dot));
    if (angle < kDeadbandRadians) return current;

    const float t     = glm::smoothstep(0.05f, 0.35f, angle);
    const float alpha = glm::mix(kSmoothingAlpha, kFastSmoothingAlpha, t);
    current = glm::normalize(glm::slerp(current, nt, alpha));
    return current;
}

void SensorManager::autoRecalibrate(glm::quat& calibRef,
                                     glm::quat& lastQ,
                                     float& stationaryTimer,
                                     const glm::quat& current) const
{
    const float angle = 2.0f * std::acos(
        std::clamp(std::abs(glm::dot(lastQ, current)), 0.0f, 1.0f));

    if (angle < kStationaryThreshold) {
        stationaryTimer += 16.0f;
        if (stationaryTimer > kStationaryTimeMs)
            calibRef = glm::normalize(glm::slerp(calibRef, current, kDriftCorrAlpha));
    } else {
        stationaryTimer = 0.0f;
    }
    lastQ = current;
}

// ── setters ──────────────────────────────────────────────────────────────────
void SensorManager::setLFAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex1);  updateSensorQuat(sensorQuatLFA, q); activeLFA   = true; lastReceivedLFA   = std::chrono::steady_clock::now(); }
void SensorManager::setRFAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex2);  updateSensorQuat(sensorQuatRFA, q); activeRFA   = true; lastReceivedRFA   = std::chrono::steady_clock::now(); }
void SensorManager::setLUAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex3);  updateSensorQuat(sensorQuat3,   q); activeLUA   = true; lastReceivedLUA   = std::chrono::steady_clock::now(); }
void SensorManager::setRUAQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex4);  updateSensorQuat(sensorQuatRUA, q); activeRUA   = true; lastReceivedRUA   = std::chrono::steady_clock::now(); }
void SensorManager::setLTHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex5);  updateSensorQuat(sensorQuat5,   q); activeLTH   = true; lastReceivedLTH   = std::chrono::steady_clock::now(); }
void SensorManager::setLSHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex6);  updateSensorQuat(sensorQuat6,   q); activeLSH   = true; lastReceivedLSH   = std::chrono::steady_clock::now(); }
void SensorManager::setRTHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex7);  updateSensorQuat(sensorQuat7,   q); activeRTH   = true; lastReceivedRTH   = std::chrono::steady_clock::now(); }
void SensorManager::setRSHQuat(const glm::quat& q)  { std::lock_guard<std::mutex> l(quatMutex8);  updateSensorQuat(sensorQuat8,   q); activeRSH   = true; lastReceivedRSH   = std::chrono::steady_clock::now(); }
void SensorManager::setHipsQuat(const glm::quat& q) { std::lock_guard<std::mutex> l(quatMutex9);  updateSensorQuat(sensorQuat9,   q); activeHips  = true; lastReceivedHips  = std::chrono::steady_clock::now(); }
void SensorManager::setChestQuat(const glm::quat& q){ std::lock_guard<std::mutex> l(quatMutex10); updateSensorQuat(sensorQuat10,  q); activeChest = true; lastReceivedChest = std::chrono::steady_clock::now(); }

// ── getters ──────────────────────────────────────────────────────────────────
glm::quat SensorManager::getLFAQuat()  const { std::lock_guard<std::mutex> l(quatMutex1);  return sensorQuatLFA; }
glm::quat SensorManager::getRFAQuat()  const { std::lock_guard<std::mutex> l(quatMutex2);  return sensorQuatRFA; }
glm::quat SensorManager::getLUAQuat()  const { std::lock_guard<std::mutex> l(quatMutex3);  return sensorQuat3;   }
glm::quat SensorManager::getRUAQuat()  const { std::lock_guard<std::mutex> l(quatMutex4);  return sensorQuatRUA; }
glm::quat SensorManager::getLTHQuat()  const { std::lock_guard<std::mutex> l(quatMutex5);  return sensorQuat5;   }
glm::quat SensorManager::getLSHQuat()  const { std::lock_guard<std::mutex> l(quatMutex6);  return sensorQuat6;   }
glm::quat SensorManager::getRTHQuat()  const { std::lock_guard<std::mutex> l(quatMutex7);  return sensorQuat7;   }
glm::quat SensorManager::getRSHQuat()  const { std::lock_guard<std::mutex> l(quatMutex8);  return sensorQuat8;   }
glm::quat SensorManager::getHipsQuat() const { std::lock_guard<std::mutex> l(quatMutex9);  return sensorQuat9;   }
glm::quat SensorManager::getChestQuat()const { std::lock_guard<std::mutex> l(quatMutex10); return sensorQuat10;  }

// ── active flags ─────────────────────────────────────────────────────────────
static bool notTimedOut(const std::chrono::steady_clock::time_point& t, int timeoutMs) {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t).count();
    return ms < timeoutMs;
}

bool SensorManager::isLFAActive()   const { std::lock_guard<std::mutex> l(quatMutex1);  return activeLFA   && notTimedOut(lastReceivedLFA,   kSensorTimeoutMs); }
bool SensorManager::isRFAActive()   const { std::lock_guard<std::mutex> l(quatMutex2);  return activeRFA   && notTimedOut(lastReceivedRFA,   kSensorTimeoutMs); }
bool SensorManager::isLUAActive()   const { std::lock_guard<std::mutex> l(quatMutex3);  return activeLUA   && notTimedOut(lastReceivedLUA,   kSensorTimeoutMs); }
bool SensorManager::isRUAActive()   const { std::lock_guard<std::mutex> l(quatMutex4);  return activeRUA   && notTimedOut(lastReceivedRUA,   kSensorTimeoutMs); }
bool SensorManager::isLTHActive()   const { std::lock_guard<std::mutex> l(quatMutex5);  return activeLTH   && notTimedOut(lastReceivedLTH,   kSensorTimeoutMs); }
bool SensorManager::isLSHActive()   const { std::lock_guard<std::mutex> l(quatMutex6);  return activeLSH   && notTimedOut(lastReceivedLSH,   kSensorTimeoutMs); }
bool SensorManager::isRTHActive()   const { std::lock_guard<std::mutex> l(quatMutex7);  return activeRTH   && notTimedOut(lastReceivedRTH,   kSensorTimeoutMs); }
bool SensorManager::isRSHActive()   const { std::lock_guard<std::mutex> l(quatMutex8);  return activeRSH   && notTimedOut(lastReceivedRSH,   kSensorTimeoutMs); }
bool SensorManager::isHipsActive()  const { std::lock_guard<std::mutex> l(quatMutex9);  return activeHips  && notTimedOut(lastReceivedHips,  kSensorTimeoutMs); }
bool SensorManager::isChestActive() const { std::lock_guard<std::mutex> l(quatMutex10); return activeChest && notTimedOut(lastReceivedChest, kSensorTimeoutMs); }

// ── calibration ──────────────────────────────────────────────────────────────
void SensorManager::calibrateLFA()  { auto q = getLFAQuat();  std::lock_guard<std::mutex> l(calibMutex1);  calibrationReferenceLFA = glm::normalize(q); hasSmoothedCorrectedLFA = false; stationaryTimerLFA = 0; hasSmoothedLocalLFA = false; std::cout << "Calibrated L_FA\n"; }
void SensorManager::calibrateRFA()  { auto q = getRFAQuat();  std::lock_guard<std::mutex> l(calibMutex2);  calibrationReferenceRFA = glm::normalize(q); hasSmoothedCorrectedRFA = false; stationaryTimerRFA = 0; hasSmoothedLocalRFA = false; std::cout << "Calibrated R_FA\n"; }
void SensorManager::calibrateLUA()  { auto q = getLUAQuat();  std::lock_guard<std::mutex> l(calibMutex3);  calibrationReference3 = glm::normalize(q); hasSmoothedCorrected3 = false; stationaryTimer3 = 0; hasSmoothedLocalLUA = false; std::cout << "Calibrated L_UA\n"; }
void SensorManager::calibrateRUA()  { auto q = getRUAQuat();  std::lock_guard<std::mutex> l(calibMutex4);  calibrationReferenceRUA = glm::normalize(q); hasSmoothedCorrectedRUA = false; stationaryTimerRUA = 0; hasSmoothedLocalRUA = false; std::cout << "Calibrated R_UA\n"; }
void SensorManager::calibrateLTH()  { auto q = getLTHQuat();  std::lock_guard<std::mutex> l(calibMutex5);  calibrationReference5 = glm::normalize(q); hasSmoothedCorrected5 = false; stationaryTimer5 = 0; hasSmoothedLocalLTH = false; std::cout << "Calibrated L_TH\n"; }
void SensorManager::calibrateLSH()  { auto q = getLSHQuat();  std::lock_guard<std::mutex> l(calibMutex6);  calibrationReference6 = glm::normalize(q); hasSmoothedCorrected6 = false; stationaryTimer6 = 0; hasSmoothedLocalLSH = false; std::cout << "Calibrated L_SH\n"; }
void SensorManager::calibrateRTH()  { auto q = getRTHQuat();  std::lock_guard<std::mutex> l(calibMutex7);  calibrationReference7 = glm::normalize(q); hasSmoothedCorrected7 = false; stationaryTimer7 = 0; hasSmoothedLocalRTH = false; std::cout << "Calibrated R_TH\n"; }
void SensorManager::calibrateRSH()  { auto q = getRSHQuat();  std::lock_guard<std::mutex> l(calibMutex8);  calibrationReference8 = glm::normalize(q); hasSmoothedCorrected8 = false; stationaryTimer8 = 0; hasSmoothedLocalRSH = false; std::cout << "Calibrated R_SH\n"; }
void SensorManager::calibrateHips() { auto q = getHipsQuat(); std::lock_guard<std::mutex> l(calibMutex9);  calibrationReference9 = glm::normalize(q); hasSmoothedCorrected9 = false; stationaryTimer9 = 0; std::cout << "Calibrated HIPS\n"; }
void SensorManager::calibrateChest(){ auto q = getChestQuat();std::lock_guard<std::mutex> l(calibMutex10); calibrationReference10 = glm::normalize(q); hasSmoothedCorrected10 = false; stationaryTimer10 = 0; std::cout << "Calibrated CHEST\n"; }

void SensorManager::toggleQuaternionConvention()
{
    int mode = (quaternionConvention.load() + 1) % 4;
    quaternionConvention.store(mode);
    std::scoped_lock lock(calibMutex1, calibMutex2, calibMutex3, calibMutex4,
                          calibMutex5, calibMutex6, calibMutex7, calibMutex8,
                          calibMutex9, calibMutex10);
    hasSmoothedCorrectedLFA = hasSmoothedCorrectedRFA = false;
    hasSmoothedCorrected3   = hasSmoothedCorrectedRUA = false;
    hasSmoothedCorrected5   = hasSmoothedCorrected6   = false;
    hasSmoothedCorrected7   = hasSmoothedCorrected8   = false;
    hasSmoothedCorrected9   = hasSmoothedCorrected10  = false;
    hasSmoothedLocalLFA = hasSmoothedLocalRFA = false;
    hasSmoothedLocalLUA = hasSmoothedLocalRUA = false;
    hasSmoothedLocalLTH = hasSmoothedLocalRTH = false;
    hasSmoothedLocalLSH = hasSmoothedLocalRSH = false;
    int displayMode = getQuaternionMode();
    std::cout << "Quaternion mode " << displayMode << "/8. Recalibrate.\n";
}

void SensorManager::toggleVerticalOffset()
{
    verticalMode.store(!verticalMode.load());
    std::scoped_lock lock(calibMutex1, calibMutex2, calibMutex3, calibMutex4,
                          calibMutex5, calibMutex6, calibMutex7, calibMutex8,
                          calibMutex9, calibMutex10);
    hasSmoothedCorrectedLFA = hasSmoothedCorrectedRFA = false;
    hasSmoothedCorrected3   = hasSmoothedCorrectedRUA = false;
    hasSmoothedCorrected5   = hasSmoothedCorrected6   = false;
    hasSmoothedCorrected7   = hasSmoothedCorrected8   = false;
    hasSmoothedCorrected9   = hasSmoothedCorrected10  = false;
    hasSmoothedLocalLFA = hasSmoothedLocalRFA = false;
    hasSmoothedLocalLUA = hasSmoothedLocalRUA = false;
    hasSmoothedLocalLTH = hasSmoothedLocalRTH = false;
    hasSmoothedLocalLSH = hasSmoothedLocalRSH = false;
    int displayMode = getQuaternionMode();
    std::cout << "Switched to " << (verticalMode.load() ? "vertical" : "horizontal")
              << " mount. Current mode " << displayMode << "/8. Recalibrate.\n";
}

void SensorManager::togglePlacementGuideMode()
{
    placementGuideMode.store(!placementGuideMode.load());
    std::cout << "Placement guide " << (placementGuideMode.load() ? "ON" : "OFF") << "\n";
}

bool SensorManager::isPlacementGuideMode() const
{
    return placementGuideMode.load();
}

int SensorManager::getQuaternionMode() const
{
    return quaternionConvention.load() + 1 + (verticalMode.load() ? 4 : 0);
}

// ── corrected getters (now with full hierarchy) ──────────────────────────────

glm::quat SensorManager::getCorrectedLFAQuat() const
{
    glm::quat parentRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex3);
        std::lock_guard<std::mutex> lc(calibMutex3);
        parentRawCorrected = computeCorrectedQuat(sensorQuat3, calibrationReference3);
    }
    glm::quat childRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex1);
        std::lock_guard<std::mutex> lc(calibMutex1);
        autoRecalibrate(calibrationReferenceLFA, lastQLFA, stationaryTimerLFA, sensorQuatLFA);
        childRawCorrected = computeCorrectedQuat(sensorQuatLFA, calibrationReferenceLFA);
    }
    glm::quat rawLocal = glm::normalize(glm::inverse(parentRawCorrected) * childRawCorrected);
    glm::quat smoothedLocal;
    {
        std::lock_guard<std::mutex> lc(calibMutex1);
        smoothCorrectedQuat(smoothedLocalLFA, hasSmoothedLocalLFA, rawLocal);
        smoothedLocal = smoothedLocalLFA;
    }
    glm::quat parentSmoothed = getCorrectedLUAQuat();
    return glm::normalize(parentSmoothed * smoothedLocal);
}

glm::quat SensorManager::getCorrectedRFAQuat() const
{
    glm::quat parentRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex4);
        std::lock_guard<std::mutex> lc(calibMutex4);
        parentRawCorrected = computeCorrectedQuat(sensorQuatRUA, calibrationReferenceRUA);
    }
    glm::quat childRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex2);
        std::lock_guard<std::mutex> lc(calibMutex2);
        autoRecalibrate(calibrationReferenceRFA, lastQRFA, stationaryTimerRFA, sensorQuatRFA);
        childRawCorrected = computeCorrectedQuat(sensorQuatRFA, calibrationReferenceRFA);
    }
    glm::quat rawLocal = glm::normalize(glm::inverse(parentRawCorrected) * childRawCorrected);
    glm::quat smoothedLocal;
    {
        std::lock_guard<std::mutex> lc(calibMutex2);
        smoothCorrectedQuat(smoothedLocalRFA, hasSmoothedLocalRFA, rawLocal);
        smoothedLocal = smoothedLocalRFA;
    }
    glm::quat parentSmoothed = getCorrectedRUAQuat();
    return glm::normalize(parentSmoothed * smoothedLocal);
}

glm::quat SensorManager::getCorrectedLUAQuat() const
{
    // parent = Chest
    glm::quat parentRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex10);
        std::lock_guard<std::mutex> lc(calibMutex10);
        parentRawCorrected = computeCorrectedQuat(sensorQuat10, calibrationReference10);
    }
    glm::quat childRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex3);
        std::lock_guard<std::mutex> lc(calibMutex3);
        autoRecalibrate(calibrationReference3, lastQ3, stationaryTimer3, sensorQuat3);
        childRawCorrected = computeCorrectedQuat(sensorQuat3, calibrationReference3);
    }
    glm::quat rawLocal = glm::normalize(glm::inverse(parentRawCorrected) * childRawCorrected);
    glm::quat smoothedLocal;
    {
        std::lock_guard<std::mutex> lc(calibMutex3);
        smoothCorrectedQuat(smoothedLocalLUA, hasSmoothedLocalLUA, rawLocal);
        smoothedLocal = smoothedLocalLUA;
    }
    glm::quat parentSmoothed = getCorrectedChestQuat();
    return glm::normalize(parentSmoothed * smoothedLocal);
}

glm::quat SensorManager::getCorrectedRUAQuat() const
{
    glm::quat parentRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex10);
        std::lock_guard<std::mutex> lc(calibMutex10);
        parentRawCorrected = computeCorrectedQuat(sensorQuat10, calibrationReference10);
    }
    glm::quat childRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex4);
        std::lock_guard<std::mutex> lc(calibMutex4);
        autoRecalibrate(calibrationReferenceRUA, lastQRUA, stationaryTimerRUA, sensorQuatRUA);
        childRawCorrected = computeCorrectedQuat(sensorQuatRUA, calibrationReferenceRUA);
    }
    glm::quat rawLocal = glm::normalize(glm::inverse(parentRawCorrected) * childRawCorrected);
    glm::quat smoothedLocal;
    {
        std::lock_guard<std::mutex> lc(calibMutex4);
        smoothCorrectedQuat(smoothedLocalRUA, hasSmoothedLocalRUA, rawLocal);
        smoothedLocal = smoothedLocalRUA;
    }
    glm::quat parentSmoothed = getCorrectedChestQuat();
    return glm::normalize(parentSmoothed * smoothedLocal);
}

glm::quat SensorManager::getCorrectedLTHQuat() const
{
    // parent = Hips
    glm::quat parentRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex9);
        std::lock_guard<std::mutex> lc(calibMutex9);
        parentRawCorrected = computeCorrectedQuat(sensorQuat9, calibrationReference9);
    }
    glm::quat childRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex5);
        std::lock_guard<std::mutex> lc(calibMutex5);
        autoRecalibrate(calibrationReference5, lastQ5, stationaryTimer5, sensorQuat5);
        childRawCorrected = computeCorrectedQuat(sensorQuat5, calibrationReference5);
    }
    glm::quat rawLocal = glm::normalize(glm::inverse(parentRawCorrected) * childRawCorrected);
    glm::quat smoothedLocal;
    {
        std::lock_guard<std::mutex> lc(calibMutex5);
        smoothCorrectedQuat(smoothedLocalLTH, hasSmoothedLocalLTH, rawLocal);
        smoothedLocal = smoothedLocalLTH;
    }
    glm::quat parentSmoothed = getCorrectedHipsQuat();
    return glm::normalize(parentSmoothed * smoothedLocal);
}

glm::quat SensorManager::getCorrectedLSHQuat() const
{
    glm::quat parentRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex5);
        std::lock_guard<std::mutex> lc(calibMutex5);
        parentRawCorrected = computeCorrectedQuat(sensorQuat5, calibrationReference5);
    }
    glm::quat childRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex6);
        std::lock_guard<std::mutex> lc(calibMutex6);
        autoRecalibrate(calibrationReference6, lastQ6, stationaryTimer6, sensorQuat6);
        childRawCorrected = computeCorrectedQuat(sensorQuat6, calibrationReference6);
    }
    glm::quat rawLocal = glm::normalize(glm::inverse(parentRawCorrected) * childRawCorrected);
    glm::quat smoothedLocal;
    {
        std::lock_guard<std::mutex> lc(calibMutex6);
        smoothCorrectedQuat(smoothedLocalLSH, hasSmoothedLocalLSH, rawLocal);
        smoothedLocal = smoothedLocalLSH;
    }
    glm::quat parentSmoothed = getCorrectedLTHQuat();
    return glm::normalize(parentSmoothed * smoothedLocal);
}

glm::quat SensorManager::getCorrectedRTHQuat() const
{
    glm::quat parentRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex9);
        std::lock_guard<std::mutex> lc(calibMutex9);
        parentRawCorrected = computeCorrectedQuat(sensorQuat9, calibrationReference9);
    }
    glm::quat childRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex7);
        std::lock_guard<std::mutex> lc(calibMutex7);
        autoRecalibrate(calibrationReference7, lastQ7, stationaryTimer7, sensorQuat7);
        childRawCorrected = computeCorrectedQuat(sensorQuat7, calibrationReference7);
    }
    glm::quat rawLocal = glm::normalize(glm::inverse(parentRawCorrected) * childRawCorrected);
    glm::quat smoothedLocal;
    {
        std::lock_guard<std::mutex> lc(calibMutex7);
        smoothCorrectedQuat(smoothedLocalRTH, hasSmoothedLocalRTH, rawLocal);
        smoothedLocal = smoothedLocalRTH;
    }
    glm::quat parentSmoothed = getCorrectedHipsQuat();
    return glm::normalize(parentSmoothed * smoothedLocal);
}

glm::quat SensorManager::getCorrectedRSHQuat() const
{
    glm::quat parentRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex7);
        std::lock_guard<std::mutex> lc(calibMutex7);
        parentRawCorrected = computeCorrectedQuat(sensorQuat7, calibrationReference7);
    }
    glm::quat childRawCorrected;
    {
        std::lock_guard<std::mutex> l(quatMutex8);
        std::lock_guard<std::mutex> lc(calibMutex8);
        autoRecalibrate(calibrationReference8, lastQ8, stationaryTimer8, sensorQuat8);
        childRawCorrected = computeCorrectedQuat(sensorQuat8, calibrationReference8);
    }
    glm::quat rawLocal = glm::normalize(glm::inverse(parentRawCorrected) * childRawCorrected);
    glm::quat smoothedLocal;
    {
        std::lock_guard<std::mutex> lc(calibMutex8);
        smoothCorrectedQuat(smoothedLocalRSH, hasSmoothedLocalRSH, rawLocal);
        smoothedLocal = smoothedLocalRSH;
    }
    glm::quat parentSmoothed = getCorrectedRTHQuat();
    return glm::normalize(parentSmoothed * smoothedLocal);
}

glm::quat SensorManager::getCorrectedHipsQuat() const
{
    auto q = getHipsQuat();
    std::lock_guard<std::mutex> l(calibMutex9);
    autoRecalibrate(calibrationReference9, lastQ9, stationaryTimer9, q);
    return smoothCorrectedQuat(smoothedCorrected9, hasSmoothedCorrected9, computeCorrectedQuat(q, calibrationReference9));
}

glm::quat SensorManager::getCorrectedChestQuat() const
{
    auto q = getChestQuat();
    std::lock_guard<std::mutex> l(calibMutex10);
    autoRecalibrate(calibrationReference10, lastQ10, stationaryTimer10, q);
    return smoothCorrectedQuat(smoothedCorrected10, hasSmoothedCorrected10, computeCorrectedQuat(q, calibrationReference10));
}