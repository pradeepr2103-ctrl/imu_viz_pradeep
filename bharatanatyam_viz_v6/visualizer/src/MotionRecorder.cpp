#include "MotionRecorder.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace mocap {

void MotionRecorder::startRecording(){
    recording_=true; recordingStartTime_=0.0; buffer_.clear();
    std::cout<<"[Rec] Recording started\n";
}

bool MotionRecorder::stopRecording(const std::string& filepath){
    if(!recording_) return false;
    recording_=false;
    std::ofstream f(filepath);
    if(!f.is_open()){std::cerr<<"Cannot open "<<filepath<<"\n";return false;}
    f<<"timestamp";
    for(int i=0;i<kNumSensors;i++)
        f<<",qw"<<i<<",qx"<<i<<",qy"<<i<<",qz"<<i;
    f<<"\n";
    for(auto& fr:buffer_){
        f<<fr.timestamp;
        for(int i=0;i<kNumSensors;i++)
            f<<","<<fr.pose[i].w<<","<<fr.pose[i].x
             <<","<<fr.pose[i].y<<","<<fr.pose[i].z;
        f<<"\n";
    }
    std::cout<<"[Rec] Saved "<<buffer_.size()<<" frames to "<<filepath<<"\n";
    return true;
}

void MotionRecorder::onSensorUpdate(int id,const Quat& q){
    if(id>=0&&id<kNumSensors) livePose_[id]=q;
}

void MotionRecorder::update(double currentTime){
    if(recording_){
        if(recordingStartTime_==0.0) recordingStartTime_=currentTime;
        RecordedFrame fr;
        fr.timestamp=currentTime-recordingStartTime_;
        fr.pose=livePose_;
        buffer_.push_back(fr);
    }
    if(playing_){
        double dt=currentTime-lastUpdateTime_;
        if(dt>0&&dt<0.1) playbackTime_+=dt;
        if(!buffer_.empty()&&playbackTime_>=buffer_.back().timestamp){
            playbackTime_=buffer_.back().timestamp;
            playing_=false;
            std::cout<<"[Rec] Playback finished\n";
        }
        sampleAt(playbackTime_);
    }
    lastUpdateTime_=currentTime;
}

bool MotionRecorder::loadRecording(const std::string& filepath){
    std::ifstream f(filepath);
    if(!f.is_open()){std::cerr<<"Cannot open "<<filepath<<"\n";return false;}
    buffer_.clear();
    std::string line; std::getline(f,line);
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
    std::cout<<"[Rec] Loaded "<<buffer_.size()<<" frames from "<<filepath<<"\n";
    return !buffer_.empty();
}

void MotionRecorder::play(){
    if(buffer_.empty()){std::cerr<<"Nothing to play\n";return;}
    playing_=true; std::cout<<"[Rec] Playing\n";
}
void MotionRecorder::pause(){ playing_=false; }
void MotionRecorder::stop(){ playing_=false; playbackTime_=0.0;
    if(!buffer_.empty()) currentPose_=buffer_[0].pose; }
void MotionRecorder::seek(double t){
    playbackTime_=std::max(0.0,std::min(t,duration()));
    sampleAt(playbackTime_);
}

void MotionRecorder::sampleAt(double t){
    if(buffer_.empty()) return;
    size_t lo=0,hi=buffer_.size()-1;
    while(lo<hi){size_t mid=(lo+hi)/2;
        if(buffer_[mid].timestamp<t) lo=mid+1; else hi=mid;}
    if(lo>0){
        double a=std::abs(buffer_[lo-1].timestamp-t);
        double b=std::abs(buffer_[lo].timestamp-t);
        if(a<b) lo=lo-1;
    }
    currentPose_=buffer_[lo].pose;
}

} // namespace mocap
