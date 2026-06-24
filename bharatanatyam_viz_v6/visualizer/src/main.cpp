#include "Renderer.h"
#include "Network.h"
#include "MotionRecorder.h"
#include "Skeleton.h"
#include <iostream>
#include <cmath>
#include <string>
#include <sys/stat.h>

struct KeyState {
    bool wasDown[512]={};
    bool pressed(GLFWwindow* w,int key){
        bool down=(glfwGetKey(w,key)==GLFW_PRESS);
        bool edge=down&&!wasDown[key];
        wasDown[key]=down; return edge;
    }
};

int main(){
    mkdir("recordings",0755);

    mocap::Renderer renderer("visualizer/assets/models/human.glb");
    if(renderer.shouldClose()){
        std::cerr<<"Renderer init failed\n"; return 1;
    }

    mocap::UdpReceiver udp;
    bool udpOk=udp.start(5005);
    if(!udpOk)
        std::cerr<<"UDP failed — running with identity pose\n";

    mocap::MotionRecorder teacherRec, studentRec;

    enum class Mode { Live, RecordTeacher, RecordStudent, Compare };
    Mode mode=Mode::Live;

    std::array<mocap::Quat,mocap::kNumSensors> livePose;
    for(auto& q:livePose) q={1,0,0,0};

    KeyState keys;
    GLFWwindow* win=renderer.window();

    // Build sensor name string for startup banner
    std::string sensorList;
    for(int i=0;i<mocap::kNumSensors;i++){
        if(i>0) sensorList+=", ";
        sensorList+=mocap::kSensorMap[i].displayName;
    }

    std::cout<<"\n=== IMU Visualizer - "<<sensorList<<" ===\n"
               "  Live view: model (left 75%) + sensor orientation panels (right 25%)\n"
               "\n"
               "=== Recording & Comparison Controls ===\n"
               "  T     = start recording TEACHER pose\n"
               "  S     = start recording STUDENT pose\n"
               "  R     = stop & save CSV to recordings/\n"
               "  P     = play side-by-side comparison (teacher=blue | student=red)\n"
               "  Space = pause / resume comparison playback\n"
               "  Left  = scrub back 1 second\n"
               "  Right = scrub forward 1 second\n"
               "  ESC   = quit\n"
               "=========================================\n\n";

    while(!renderer.shouldClose()){
        double now=glfwGetTime();
        glfwPollEvents();

        if(keys.pressed(win,GLFW_KEY_ESCAPE))
            glfwSetWindowShouldClose(win,GLFW_TRUE);

        if(keys.pressed(win,GLFW_KEY_T)&&mode==Mode::Live){
            teacherRec.startRecording(); mode=Mode::RecordTeacher;
            std::cout<<"[T] Recording TEACHER — press R to stop\n";
        }
        if(keys.pressed(win,GLFW_KEY_S)&&mode==Mode::Live){
            studentRec.startRecording(); mode=Mode::RecordStudent;
            std::cout<<"[S] Recording STUDENT — press R to stop\n";
        }
        if(keys.pressed(win,GLFW_KEY_R)){
            if(mode==Mode::RecordTeacher){
                teacherRec.stopRecording("recordings/teacher.csv"); mode=Mode::Live;
            } else if(mode==Mode::RecordStudent){
                studentRec.stopRecording("recordings/student.csv"); mode=Mode::Live;
            } else if(mode==Mode::Compare){
                teacherRec.stop(); studentRec.stop(); mode=Mode::Live;
            }
        }
        if(keys.pressed(win,GLFW_KEY_P)&&mode==Mode::Live){
            bool tOk=teacherRec.loadRecording("recordings/teacher.csv");
            bool sOk=studentRec.loadRecording("recordings/student.csv");
            if(tOk&&sOk){ teacherRec.play(); studentRec.play(); mode=Mode::Compare; }
            else std::cout<<"[P] Record teacher (T) and student (S) first\n";
        }
        if(keys.pressed(win,GLFW_KEY_SPACE)&&mode==Mode::Compare){
            if(teacherRec.isPlaying()){teacherRec.pause();studentRec.pause();}
            else{teacherRec.play();studentRec.play();}
        }
        if(keys.pressed(win,GLFW_KEY_LEFT)&&mode==Mode::Compare){
            double t=std::max(0.0,teacherRec.currentTime()-1.0);
            teacherRec.seek(t); studentRec.seek(t);
        }
        if(keys.pressed(win,GLFW_KEY_RIGHT)&&mode==Mode::Compare){
            teacherRec.seek(teacherRec.currentTime()+1.0);
            studentRec.seek(studentRec.currentTime()+1.0);
        }

        // Feed live UDP data into pose array
        if(udpOk){
            auto samples=udp.getAll();
            for(int i=0;i<10;i++)
                if(samples[i].received)
                    livePose[i]={samples[i].qw,samples[i].qx,
                                 samples[i].qy,samples[i].qz};
        }
        for(int i=0;i<mocap::kNumSensors;i++){
            if(mode==Mode::RecordTeacher) teacherRec.onSensorUpdate(i,livePose[i]);
            if(mode==Mode::RecordStudent) studentRec.onSensorUpdate(i,livePose[i]);
        }
        teacherRec.update(now);
        studentRec.update(now);

        switch(mode){
            case Mode::Live:
            case Mode::RecordTeacher:
            case Mode::RecordStudent:
                renderer.renderLive(livePose);
                renderer.drawHUD(
                    teacherRec.isRecording()||studentRec.isRecording(),
                    false,false,0.0,
                    mode==Mode::RecordTeacher?"TEACHER":
                    mode==Mode::RecordStudent?"STUDENT":"");
                break;
            case Mode::Compare:
                renderer.renderCompare(teacherRec.getPose(),studentRec.getPose());
                renderer.drawHUD(false,teacherRec.isPlaying(),true,
                                 teacherRec.currentTime(),"");
                break;
        }
    }
    udp.stop();
    return 0;
}
