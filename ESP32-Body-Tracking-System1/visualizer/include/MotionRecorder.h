#pragma once
#include <string>
#include <vector>
#include <array>
#include "MathTypes.h"

namespace mocap {

constexpr int kNumSensors = 10;

struct RecordedFrame {
    double timestamp;
    std::array<Quat, kNumSensors> pose;
};

class MotionRecorder {
public:
    void startRecording();
    bool stopRecording(const std::string& filepath);
    bool loadRecording(const std::string& filepath);

    void update(double currentTime);
    void onSensorUpdate(int sensorId, const Quat& q);
    const std::array<Quat, kNumSensors>& getPose() const { return currentPose_; }

    // Playback
    void play();
    void pause();
    void stop();
    void seek(double t);
    bool isPlaying() const { return playing_; }
    double currentTime() const { return playbackTime_; }

private:
    std::array<Quat, kNumSensors> currentPose_;
    std::array<Quat, kNumSensors> livePose_;   // buffer for recording
    std::vector<RecordedFrame> buffer_;
    double playbackTime_ = 0.0;
    bool playing_ = false;
    bool recording_ = false;
    double recordingStartTime_ = 0.0;

    void sampleAt(double t);
};

} // namespace mocap