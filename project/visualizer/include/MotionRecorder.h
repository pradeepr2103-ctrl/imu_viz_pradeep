#pragma once
#include <string>
#include <vector>
#include <array>
#include "MathTypes.h"
#include "Skeleton.h"

namespace mocap {

struct RecordedFrame {
    double                        timestamp;
    std::array<Quat, kNumSensors> pose;
};

class MotionRecorder {
public:
    // Recording
    void startRecording();
    bool stopRecording(const std::string& filepath);

    // Live sensor feed (call every frame)
    void onSensorUpdate(int sensorId, const Quat& q);

    // Update (advance playback clock)
    void update(double currentTime);

    // Playback
    bool loadRecording(const std::string& filepath);
    void play();
    void pause();
    void stop();
    void seek(double t);

    // Getters
    const std::array<Quat,kNumSensors>& getPose()    const { return currentPose_; }
    bool   isPlaying()    const { return playing_; }
    bool   isRecording()  const { return recording_; }
    double currentTime()  const { return playbackTime_; }
    double duration()     const { return buffer_.empty()?0.0:buffer_.back().timestamp; }
    size_t frameCount()   const { return buffer_.size(); }

private:
    std::array<Quat,kNumSensors> livePose_;
    std::array<Quat,kNumSensors> currentPose_;
    std::vector<RecordedFrame>   buffer_;

    bool   recording_          = false;
    bool   playing_            = false;
    double recordingStartTime_ = 0.0;
    double playbackTime_       = 0.0;
    double lastUpdateTime_     = 0.0;

    void sampleAt(double t);
};

} // namespace mocap
