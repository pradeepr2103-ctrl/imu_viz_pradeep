#include "MotionRecorder.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <cassert>

namespace mocap {

void MotionRecorder::startRecording() {
    recording_ = true;
    buffer_.clear();
    recordingStartTime_ = 0.0;
    std::cout << "Recording started" << std::endl;
}

bool MotionRecorder::stopRecording(const std::string& filepath) {
    if (!recording_) return false;
    recording_ = false;

    std::ofstream f(filepath);
    if (!f.is_open()) return false;
    // Write CSV header
    f << "timestamp";
    for (int i = 0; i < kNumSensors; ++i)
        f << ",qw" << i << ",qx" << i << ",qy" << i << ",qz" << i;
    f << "\n";
    for (auto& frame : buffer_) {
        f << frame.timestamp;
        for (int i = 0; i < kNumSensors; ++i)
            f << "," << frame.pose[i].w << "," << frame.pose[i].x
              << "," << frame.pose[i].y << "," << frame.pose[i].z;
        f << "\n";
    }
    f.close();
    std::cout << "Saved " << buffer_.size() << " frames to " << filepath << std::endl;
    buffer_.clear();
    return true;
}

void MotionRecorder::onSensorUpdate(int sensorId, const Quat& q) {
    if (sensorId < 0 || sensorId >= kNumSensors) return;
    livePose_[sensorId] = q;
}

void MotionRecorder::update(double currentTime) {
    if (recording_) {
        if (recordingStartTime_ == 0.0) recordingStartTime_ = currentTime;
        RecordedFrame frame;
        frame.timestamp = currentTime - recordingStartTime_;
        frame.pose = livePose_;
        buffer_.push_back(frame);
    }
    if (playing_) {
        // Advance time – approx. 60fps, accurate delta would be better
        playbackTime_ += 1.0 / 60.0;
        sampleAt(playbackTime_);
    }
}

void MotionRecorder::play() {
    if (buffer_.empty()) return;
    playing_ = true;
    playbackTime_ = 0.0;
}

void MotionRecorder::pause() { playing_ = false; }
void MotionRecorder::stop() { playing_ = false; playbackTime_ = 0.0; }

void MotionRecorder::seek(double t) {
    if (t < 0) t = 0;
    playbackTime_ = t;
    sampleAt(t);
}

void MotionRecorder::sampleAt(double t) {
    if (buffer_.empty()) return;
    // Nearest neighbour (simple and safe)
    size_t best = 0;
    double bestDiff = 1e9;
    for (size_t i = 0; i < buffer_.size(); ++i) {
        double diff = std::abs(buffer_[i].timestamp - t);
        if (diff < bestDiff) { bestDiff = diff; best = i; }
    }
    currentPose_ = buffer_[best].pose;
}

bool MotionRecorder::loadRecording(const std::string& filepath) {
    std::ifstream f(filepath);
    if (!f.is_open()) return false;
    buffer_.clear();
    std::string line;
    std::getline(f, line); // skip header
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        RecordedFrame frame;
        char comma;
        ss >> frame.timestamp >> comma;
        for (int i = 0; i < kNumSensors; ++i) {
            ss >> frame.pose[i].w >> comma
               >> frame.pose[i].x >> comma
               >> frame.pose[i].y >> comma
               >> frame.pose[i].z;
            // trailing comma after last sensor? ignore
        }
        buffer_.push_back(frame);
    }
    std::cout << "Loaded " << buffer_.size() << " frames from " << filepath << std::endl;
    return true;
}

} // namespace mocap