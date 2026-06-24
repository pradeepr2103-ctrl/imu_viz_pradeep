#include "MotionRecorder.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace mocap {

// ── Recording ─────────────────────────────────────────────────────────────────
void MotionRecorder::startRecording(){
    recording_          = true;
    recordingStartTime_ = 0.0;  // will be set on first update() call
    buffer_.clear();
    std::cout<<"[Recorder] Recording started\n";
}

bool MotionRecorder::stopRecording(const std::string& filepath){
    if(!recording_) return false;
    recording_ = false;

    std::ofstream f(filepath);
    if(!f.is_open()){
        std::cerr<<"[Recorder] Cannot open: "<<filepath<<std::endl;
        return false;
    }

    // CSV header
    f<<"timestamp";
    for(int i=0;i<kNumSensors;i++)
        f<<",qw"<<i<<",qx"<<i<<",qy"<<i<<",qz"<<i;
    f<<"\n";

    // Data rows
    for(auto& fr : buffer_){
        f<<fr.timestamp;
        for(int i=0;i<kNumSensors;i++)
            f<<","<<fr.pose[i].w
             <<","<<fr.pose[i].x
             <<","<<fr.pose[i].y
             <<","<<fr.pose[i].z;
        f<<"\n";
    }
    f.close();

    std::cout<<"[Recorder] Saved "<<buffer_.size()
             <<" frames ("<<duration()<<"s) to "<<filepath<<"\n";
    return true;
}

void MotionRecorder::onSensorUpdate(int id, const Quat& q){
    if(id>=0 && id<kNumSensors) livePose_[id]=q;
}

void MotionRecorder::update(double currentTime){
    if(recording_){
        if(recordingStartTime_==0.0) recordingStartTime_=currentTime;
        RecordedFrame fr;
        fr.timestamp = currentTime - recordingStartTime_;
        fr.pose      = livePose_;
        buffer_.push_back(fr);
    }

    if(playing_){
        double dt = currentTime - lastUpdateTime_;
        if(dt>0 && dt<0.1) playbackTime_ += dt;
        // Clamp and stop at end
        if(!buffer_.empty() && playbackTime_ >= buffer_.back().timestamp){
            playbackTime_ = buffer_.back().timestamp;
            playing_ = false;
            std::cout<<"[Recorder] Playback finished\n";
        }
        sampleAt(playbackTime_);
    }
    lastUpdateTime_ = currentTime;
}

// ── Playback ──────────────────────────────────────────────────────────────────
bool MotionRecorder::loadRecording(const std::string& filepath){
    std::ifstream f(filepath);
    if(!f.is_open()){
        std::cerr<<"[Recorder] Cannot open: "<<filepath<<std::endl;
        return false;
    }
    buffer_.clear();
    std::string line;
    std::getline(f,line); // skip header
    while(std::getline(f,line)){
        if(line.empty()) continue;
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream ss(line);
        RecordedFrame fr;
        ss>>fr.timestamp;
        for(int i=0;i<kNumSensors;i++)
            ss>>fr.pose[i].w>>fr.pose[i].x>>fr.pose[i].y>>fr.pose[i].z;
        buffer_.push_back(fr);
    }
    std::cout<<"[Recorder] Loaded "<<buffer_.size()
             <<" frames ("<<duration()<<"s) from "<<filepath<<"\n";
    return !buffer_.empty();
}

void MotionRecorder::play(){
    if(buffer_.empty()){ std::cerr<<"[Recorder] Nothing to play\n"; return; }
    playing_=true;
    std::cout<<"[Recorder] Playing\n";
}

void MotionRecorder::pause(){
    playing_=false;
    std::cout<<"[Recorder] Paused at "<<playbackTime_<<"s\n";
}

void MotionRecorder::stop(){
    playing_=false;
    playbackTime_=0.0;
    if(!buffer_.empty()) currentPose_=buffer_[0].pose;
}

void MotionRecorder::seek(double t){
    playbackTime_=std::max(0.0, std::min(t, duration()));
    sampleAt(playbackTime_);
}

// Nearest-neighbour sample (fast, no interpolation artefacts on slow data)
void MotionRecorder::sampleAt(double t){
    if(buffer_.empty()) return;
    // Binary search for closest frame
    size_t lo=0, hi=buffer_.size()-1;
    while(lo<hi){
        size_t mid=(lo+hi)/2;
        if(buffer_[mid].timestamp < t) lo=mid+1;
        else hi=mid;
    }
    // Pick nearest of lo-1 and lo
    if(lo>0){
        double dPrev=std::abs(buffer_[lo-1].timestamp-t);
        double dCurr=std::abs(buffer_[lo].timestamp-t);
        if(dPrev<dCurr) lo=lo-1;
    }
    currentPose_=buffer_[lo].pose;
}

} // namespace mocap
