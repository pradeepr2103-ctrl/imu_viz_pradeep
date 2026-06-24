#include "Renderer.h"
#include "Network.h"
#include "MotionRecorder.h"
#include "Skeleton.h"
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <string>

// ── Key helper ────────────────────────────────────────────────────────────────
struct KeyState {
    bool wasDown[512]={};
    // Returns true only on the first frame the key is pressed
    bool pressed(GLFWwindow* w, int key){
        bool down=(glfwGetKey(w,key)==GLFW_PRESS);
        bool edge=down && !wasDown[key];
        wasDown[key]=down;
        return edge;
    }
};

int main(){
    // Ensure recordings directory exists
    std::system("mkdir -p recordings");

    // ── Init renderer (loads human.glb, creates GL window) ──────────────────
    mocap::Renderer renderer("visualizer/assets/models/human.glb");
    if(renderer.shouldClose()){
        std::cerr<<"Renderer init failed — exit.\n";
        return 1;
    }

    // ── UDP receiver ────────────────────────────────────────────────────────
    mocap::UdpReceiver udp;
    bool udpOk = udp.start(5005);
    if(!udpOk)
        std::cerr<<"UDP start failed — running with identity pose (no sensor data)\n";

    // ── Motion recorders (one per role) ─────────────────────────────────────
    mocap::MotionRecorder teacherRec, studentRec;

    // ── App state ────────────────────────────────────────────────────────────
    enum class Mode { Live, RecordTeacher, RecordStudent, Compare };
    Mode mode = Mode::Live;

    // Live pose — identity quaternions
    std::array<mocap::Quat, mocap::kNumSensors> livePose;
    for(auto& q : livePose) q={1,0,0,0};

    KeyState keys;
    GLFWwindow* win = renderer.window();

    std::cout<<"\n=== Controls ===\n"
             <<"  T       — start recording TEACHER\n"
             <<"  S       — start recording STUDENT\n"
             <<"  R       — stop recording and save CSV\n"
             <<"  P       — load CSVs and play comparison\n"
             <<"  Space   — pause / resume playback\n"
             <<"  < >     — scrub ±1 second\n"
             <<"  ESC     — quit\n"
             <<"================\n\n";

    // ── Main loop ────────────────────────────────────────────────────────────
    while(!renderer.shouldClose()){
        double now = glfwGetTime();
        glfwPollEvents();

        // ── ESC ─────────────────────────────────────────────────────────────
        if(keys.pressed(win, GLFW_KEY_ESCAPE))
            glfwSetWindowShouldClose(win, GLFW_TRUE);

        // ── T: Start recording teacher ──────────────────────────────────────
        if(keys.pressed(win, GLFW_KEY_T)){
            if(mode==Mode::Live){
                teacherRec.startRecording();
                mode=Mode::RecordTeacher;
                std::cout<<"[T] Recording TEACHER... press R to stop.\n";
            }
        }

        // ── S: Start recording student ──────────────────────────────────────
        if(keys.pressed(win, GLFW_KEY_S)){
            if(mode==Mode::Live){
                studentRec.startRecording();
                mode=Mode::RecordStudent;
                std::cout<<"[S] Recording STUDENT... press R to stop.\n";
            }
        }

        // ── R: Stop recording, save CSV ─────────────────────────────────────
        if(keys.pressed(win, GLFW_KEY_R)){
            if(mode==Mode::RecordTeacher){
                if(teacherRec.stopRecording("recordings/teacher.csv"))
                    std::cout<<"[R] Saved recordings/teacher.csv ("
                             <<teacherRec.frameCount()<<" frames)\n";
                mode=Mode::Live;
            } else if(mode==Mode::RecordStudent){
                if(studentRec.stopRecording("recordings/student.csv"))
                    std::cout<<"[R] Saved recordings/student.csv ("
                             <<studentRec.frameCount()<<" frames)\n";
                mode=Mode::Live;
            } else if(mode==Mode::Compare){
                teacherRec.stop();
                studentRec.stop();
                mode=Mode::Live;
                std::cout<<"[R] Stopped comparison, back to Live.\n";
            }
        }

        // ── P: Load CSVs and start comparison playback ──────────────────────
        if(keys.pressed(win, GLFW_KEY_P)){
            if(mode==Mode::Live){
                bool tOk = teacherRec.loadRecording("recordings/teacher.csv");
                bool sOk = studentRec.loadRecording("recordings/student.csv");
                if(tOk && sOk){
                    teacherRec.play();
                    studentRec.play();
                    mode=Mode::Compare;
                    std::cout<<"[P] Playing comparison ("
                             <<teacherRec.duration()<<"s teacher, "
                             <<studentRec.duration()<<"s student)\n";
                } else {
                    std::cout<<"[P] Missing CSV — record teacher (T) and student (S) first.\n";
                }
            }
        }

        // ── Space: Pause / resume playback ──────────────────────────────────
        if(keys.pressed(win, GLFW_KEY_SPACE)){
            if(mode==Mode::Compare){
                if(teacherRec.isPlaying()){
                    teacherRec.pause(); studentRec.pause();
                } else {
                    teacherRec.play(); studentRec.play();
                }
            }
        }

        // ── < > : Scrub ±1 s ────────────────────────────────────────────────
        if(keys.pressed(win, GLFW_KEY_LEFT) && mode==Mode::Compare){
            double t=std::max(0.0, teacherRec.currentTime()-1.0);
            teacherRec.seek(t); studentRec.seek(t);
        }
        if(keys.pressed(win, GLFW_KEY_RIGHT) && mode==Mode::Compare){
            double t=teacherRec.currentTime()+1.0;
            teacherRec.seek(t); studentRec.seek(t);
        }

        // ── Update UDP sensor data ───────────────────────────────────────────
        if(udpOk){
            auto samples=udp.getAll();
            ffor(int i=0;i<mocap::kNumSensors;i++){
                if(samples[i].received)
                    livePose[i]={samples[i].qw,samples[i].qx,
                                 samples[i].qy,samples[i].qz};
            }
        }

        // Feed live pose to whichever recorder is active
        for(int i=0;i<mocap::kNumSensors;i++){
            if(mode==Mode::RecordTeacher)
                teacherRec.onSensorUpdate(i,livePose[i]);
            else if(mode==Mode::RecordStudent)
                studentRec.onSensorUpdate(i,livePose[i]);
        }

        // Advance recorder timers
        teacherRec.update(now);
        studentRec.update(now);

        // ── Render ──────────────────────────────────────────────────────────
        switch(mode){
            case Mode::Live:
            case Mode::RecordTeacher:
            case Mode::RecordStudent:
                renderer.renderLive(livePose);
                renderer.drawHUD(
                    teacherRec.isRecording()||studentRec.isRecording(),
                    false, false, 0.0,
                    mode==Mode::RecordTeacher ? "Teacher" :
                    mode==Mode::RecordStudent ? "Student"  : "");
                break;

            case Mode::Compare:
                renderer.renderCompare(
                    teacherRec.getPose(),
                    studentRec.getPose());
                renderer.drawHUD(false,
                    teacherRec.isPlaying(),
                    true,
                    teacherRec.currentTime(), "");
                break;
        }
    }

    udp.stop();
    return 0;
}
