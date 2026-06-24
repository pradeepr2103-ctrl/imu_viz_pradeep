#include <GLFW/glfw3.h>
#include <thread>
#include <mutex>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <sys/stat.h>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

#include <GL/glut.h>
#include "udp_receiver.h"
#include "sensor_manager.h"
#include "renderer.h"
#include "input_handler.h"
#include "csv_logger.h"

// ── Pose snapshot (10 quats) ─────────────────────────────────────────────────
struct Pose {
    glm::quat lFA{1,0,0,0}, rFA{1,0,0,0};
    glm::quat lUA{1,0,0,0}, rUA{1,0,0,0};
    glm::quat lTH{1,0,0,0}, rTH{1,0,0,0};
    glm::quat lSH{1,0,0,0}, rSH{1,0,0,0};
    glm::quat hips{1,0,0,0}, chest{1,0,0,0};
};

// ── Motion recorder ──────────────────────────────────────────────────────────
struct RecordedFrame { double t; Pose pose; };

class MotionRecorder {
public:
    void startRecording()  { buf_.clear(); recStart_=0.0; recording_=true; }
    bool isRecording() const { return recording_; }

    void record(double now, const Pose& p) {
        if (!recording_) return;
        if (recStart_==0.0) recStart_=now;
        buf_.push_back({now - recStart_, p});
    }

    bool stopRecording(const std::string& path) {
        recording_=false;
        std::ofstream f(path);
        if (!f) { std::cerr<<"Cannot open "<<path<<"\n"; return false; }
        f<<"t,lFA_w,lFA_x,lFA_y,lFA_z"
          ",rFA_w,rFA_x,rFA_y,rFA_z"
          ",lUA_w,lUA_x,lUA_y,lUA_z"
          ",rUA_w,rUA_x,rUA_y,rUA_z"
          ",lTH_w,lTH_x,lTH_y,lTH_z"
          ",rTH_w,rTH_x,rTH_y,rTH_z"
          ",lSH_w,lSH_x,lSH_y,lSH_z"
          ",rSH_w,rSH_x,rSH_y,rSH_z"
          ",hips_w,hips_x,hips_y,hips_z"
          ",chest_w,chest_x,chest_y,chest_z\n";
        auto wq=[&](const glm::quat& q){
            f<<","<<q.w<<","<<q.x<<","<<q.y<<","<<q.z; };
        for (auto& fr:buf_){
            f<<fr.t;
            wq(fr.pose.lFA);  wq(fr.pose.rFA);
            wq(fr.pose.lUA);  wq(fr.pose.rUA);
            wq(fr.pose.lTH);  wq(fr.pose.rTH);
            wq(fr.pose.lSH);  wq(fr.pose.rSH);
            wq(fr.pose.hips); wq(fr.pose.chest);
            f<<"\n";
        }
        std::cout<<"[Rec] Saved "<<buf_.size()<<" frames → "<<path<<"\n";
        return true;
    }

    bool loadRecording(const std::string& path){
        buf_.clear();
        std::ifstream f(path);
        if (!f){ std::cerr<<"Cannot open "<<path<<"\n"; return false; }
        std::string line; std::getline(f,line); // skip header
        auto rq=[](std::istringstream& ss, glm::quat& q){
            char c; ss>>q.w>>c>>q.x>>c>>q.y>>c>>q.z; };
        while(std::getline(f,line)){
            if(line.empty()) continue;
            std::replace(line.begin(),line.end(),',',' ');
            std::istringstream ss(line);
            RecordedFrame fr;
            ss>>fr.t;
            auto rq2=[&](glm::quat& q){ ss>>q.w>>q.x>>q.y>>q.z; };
            rq2(fr.pose.lFA);  rq2(fr.pose.rFA);
            rq2(fr.pose.lUA);  rq2(fr.pose.rUA);
            rq2(fr.pose.lTH);  rq2(fr.pose.rTH);
            rq2(fr.pose.lSH);  rq2(fr.pose.rSH);
            rq2(fr.pose.hips); rq2(fr.pose.chest);
            buf_.push_back(fr);
        }
        std::cout<<"[Rec] Loaded "<<buf_.size()<<" frames from "<<path<<"\n";
        return !buf_.empty();
    }

    void play()  { playing_=true;  pbTime_=0.0; }
    void pause() { playing_=false; }
    void stop()  { playing_=false; pbTime_=0.0; }
    void seek(double t){ pbTime_=std::max(0.0,std::min(t,duration())); }
    bool isPlaying() const { return playing_; }
    double currentTime() const { return pbTime_; }
    double duration() const { return buf_.empty()?0.0:buf_.back().t; }

    void update(double dt){
        if(!playing_) return;
        pbTime_+=dt;
        if(pbTime_>=duration()){ pbTime_=duration(); playing_=false;
            std::cout<<"[Rec] Playback finished\n"; }
    }

    Pose getPose() const {
        if(buf_.empty()) return Pose{};
        // binary search for current time
        size_t lo=0, hi=buf_.size()-1;
        while(lo<hi){ size_t mid=(lo+hi)/2;
            if(buf_[mid].t<pbTime_) lo=mid+1; else hi=mid; }
        if(lo==0) return buf_[0].pose;
        // lerp between lo-1 and lo
        const RecordedFrame& a=buf_[lo-1];
        const RecordedFrame& b=buf_[lo];
        double span=b.t-a.t;
        float alpha=(span<1e-9)?1.0f:(float)((pbTime_-a.t)/span);
        alpha=std::max(0.0f,std::min(1.0f,alpha));
        Pose p;
        auto sl=[&](const glm::quat& qa,const glm::quat& qb){ return glm::slerp(qa,qb,alpha); };
        p.lFA  = sl(a.pose.lFA,  b.pose.lFA);
        p.rFA  = sl(a.pose.rFA,  b.pose.rFA);
        p.lUA  = sl(a.pose.lUA,  b.pose.lUA);
        p.rUA  = sl(a.pose.rUA,  b.pose.rUA);
        p.lTH  = sl(a.pose.lTH,  b.pose.lTH);
        p.rTH  = sl(a.pose.rTH,  b.pose.rTH);
        p.lSH  = sl(a.pose.lSH,  b.pose.lSH);
        p.rSH  = sl(a.pose.rSH,  b.pose.rSH);
        p.hips  = sl(a.pose.hips,  b.pose.hips);
        p.chest = sl(a.pose.chest, b.pose.chest);
        return p;
    }

private:
    std::vector<RecordedFrame> buf_;
    bool   recording_=false, playing_=false;
    double recStart_=0.0, pbTime_=0.0;
};

// ── helpers ──────────────────────────────────────────────────────────────────
static glm::quat localRelativeTo(const glm::quat& parent, const glm::quat& child){
    return glm::normalize(glm::inverse(parent)*child);
}

static Pose buildLivePose(SensorManager& sm){
    glm::quat corrLFA   = sm.getCorrectedLFAQuat();
    glm::quat corrRFA   = sm.getCorrectedRFAQuat();
    glm::quat corrLUA   = sm.getCorrectedLUAQuat();
    glm::quat corrRUA   = sm.getCorrectedRUAQuat();
    glm::quat corrLTH   = sm.getCorrectedLTHQuat();
    glm::quat corrLSH   = sm.getCorrectedLSHQuat();
    glm::quat corrRTH   = sm.getCorrectedRTHQuat();
    glm::quat corrRSH   = sm.getCorrectedRSHQuat();
    glm::quat corrHips  = sm.getCorrectedHipsQuat();
    glm::quat corrChest = sm.getCorrectedChestQuat();

    Pose p;
    p.hips  = corrHips;
    p.chest = corrChest;
    p.lUA = sm.isLUAActive() ? localRelativeTo(corrChest, corrLUA) : glm::quat(1,0,0,0);
    p.rUA = sm.isRUAActive() ? localRelativeTo(corrChest, corrRUA) : glm::quat(1,0,0,0);
    p.lFA = sm.isLFAActive() ? localRelativeTo(corrLUA,   corrLFA) : glm::quat(1,0,0,0);
    p.rFA = sm.isRFAActive() ? localRelativeTo(corrRUA,   corrRFA) : glm::quat(1,0,0,0);
    p.lTH = sm.isLTHActive() ? localRelativeTo(corrHips,  corrLTH) : glm::quat(1,0,0,0);
    p.rTH = sm.isRTHActive() ? localRelativeTo(corrHips,  corrRTH) : glm::quat(1,0,0,0);
    p.lSH = sm.isLSHActive() ? localRelativeTo(corrLTH,   corrLSH) : glm::quat(1,0,0,0);
    p.rSH = sm.isRSHActive() ? localRelativeTo(corrRTH,   corrRSH) : glm::quat(1,0,0,0);
    return p;
}

// ── Key state edge detection ──────────────────────────────────────────────────
struct EdgeKeys {
    bool wasDown[512]={};
    bool pressed(GLFWwindow* w, int key){
        bool down=(glfwGetKey(w,key)==GLFW_PRESS);
        bool edge=down&&!wasDown[key];
        wasDown[key]=down; return edge;
    }
};

// ── main ─────────────────────────────────────────────────────────────────────
int main()
{
    mkdir("recordings",0755);

    SensorManager sensorManager;
    std::thread receiver(udpReceiver, std::ref(sensorManager));

    if(!glfwInit()) return -1;

    GLFWwindow* window = glfwCreateWindow(1400,900,
        "IMU Visualizer - L_FA, R_FA, L_UA, R_UA, L_TH, L_SH, R_TH, R_SH",
        nullptr,nullptr);
    if(!window){ glfwTerminate(); return -1; }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window,[](GLFWwindow*,int w,int h){
        glViewport(0,0,w,h); });

    Renderer renderer;
    renderer.initialize();

    CsvLogger csvLogger;
    csvLogger.open();

    InputHandler inputHandler(sensorManager, csvLogger, renderer);
    glfwSetKeyCallback(window, keyCallbackDispatcher);
    glfwSetWindowUserPointer(window, &inputHandler);

    MotionRecorder teacherRec, studentRec;
    enum class Mode { Live, RecordTeacher, RecordStudent, Compare };
    Mode mode = Mode::Live;

    EdgeKeys edge;
    double lastTime = glfwGetTime();

    std::cout<<"\n=== IMU Visualizer Controls ===\n"
               "  Sensor calibration (original controls):\n"
               "    C/V/B/N = calibrate L_FA/R_FA/L_UA/R_UA\n"
               "    Z/X     = calibrate L_TH/L_SH\n"
               "    G/H     = calibrate R_TH/R_SH\n"
               "    I/O     = calibrate HIPS/CHEST\n"
               "    Space   = calibrate ALL sensors\n"
               "    M       = cycle quaternion convention\n"
               "    P       = toggle vertical mount\n"
               "    L       = toggle placement guide\n"
               "    1/2/3   = camera FRONT/BACK/SIDE\n"
               "\n"
               "  Recording & comparison:\n"
               "    T       = start recording TEACHER\n"
               "    S       = start recording STUDENT\n"
               "    R       = stop & save recording\n"
               "    F       = play side-by-side comparison\n"
               "    Tab     = pause/resume comparison\n"
               "    [ / ]   = scrub comparison ±1 second\n"
               "    ESC     = quit\n"
               "================================\n\n";

    while(!glfwWindowShouldClose(window))
    {
        double now = glfwGetTime();
        double dt  = now - lastTime;
        lastTime   = now;

        glfwPollEvents();

        // ESC to quit
        if(glfwGetKey(window,GLFW_KEY_ESCAPE)==GLFW_PRESS)
            glfwSetWindowShouldClose(window,GLFW_TRUE);

        // ── Camera keys (pass through always) ───────────────────────────────
        if(glfwGetKey(window,GLFW_KEY_1)==GLFW_PRESS)
            renderer.setCameraView(CameraView::FRONT);
        if(glfwGetKey(window,GLFW_KEY_2)==GLFW_PRESS)
            renderer.setCameraView(CameraView::BACK);
        if(glfwGetKey(window,GLFW_KEY_3)==GLFW_PRESS)
            renderer.setCameraView(CameraView::SIDE);

        // ── Recording controls (edge-triggered) ─────────────────────────────
        if(edge.pressed(window,GLFW_KEY_T) && mode==Mode::Live){
            teacherRec.startRecording(); mode=Mode::RecordTeacher;
            std::cout<<"[T] Recording TEACHER — press R to stop\n";
        }
        if(edge.pressed(window,GLFW_KEY_S) && mode==Mode::Live){
            studentRec.startRecording(); mode=Mode::RecordStudent;
            std::cout<<"[S] Recording STUDENT — press R to stop\n";
        }
        if(edge.pressed(window,GLFW_KEY_R)){
            if(mode==Mode::RecordTeacher){
                teacherRec.stopRecording("recordings/teacher.csv");
                mode=Mode::Live;
            } else if(mode==Mode::RecordStudent){
                studentRec.stopRecording("recordings/student.csv");
                mode=Mode::Live;
            } else if(mode==Mode::Compare){
                teacherRec.stop(); studentRec.stop();
                mode=Mode::Live;
            }
        }
        if(edge.pressed(window,GLFW_KEY_F) && mode==Mode::Live){
            bool tOk=teacherRec.loadRecording("recordings/teacher.csv");
            bool sOk=studentRec.loadRecording("recordings/student.csv");
            if(tOk&&sOk){ teacherRec.play(); studentRec.play(); mode=Mode::Compare; }
            else std::cout<<"[F] Record teacher (T) and student (S) first\n";
        }
        if(edge.pressed(window,GLFW_KEY_TAB) && mode==Mode::Compare){
            if(teacherRec.isPlaying()){ teacherRec.pause(); studentRec.pause(); }
            else { teacherRec.play(); studentRec.play(); }
        }
        if(edge.pressed(window,GLFW_KEY_LEFT_BRACKET) && mode==Mode::Compare){
            double t=std::max(0.0,teacherRec.currentTime()-1.0);
            teacherRec.seek(t); studentRec.seek(t);
        }
        if(edge.pressed(window,GLFW_KEY_RIGHT_BRACKET) && mode==Mode::Compare){
            teacherRec.seek(teacherRec.currentTime()+1.0);
            studentRec.seek(studentRec.currentTime()+1.0);
        }

        // ── Update playback ─────────────────────────────────────────────────
        if(mode==Mode::Compare){
            teacherRec.update(dt);
            studentRec.update(dt);
        }

        // ── Build live pose ──────────────────────────────────────────────────
        Pose live = buildLivePose(sensorManager);

        // ── Record live frames ───────────────────────────────────────────────
        if(mode==Mode::RecordTeacher) teacherRec.record(now, live);
        if(mode==Mode::RecordStudent) studentRec.record(now, live);

        // ── Update window title ──────────────────────────────────────────────
        {
            char title[512];
            if(mode==Mode::Compare && teacherRec.isPlaying())
                snprintf(title,sizeof(title),
                    "IMU Visualizer [COMPARE PLAYING %.1fs / %.1fs]  Tab=pause  [/]=scrub",
                    teacherRec.currentTime(), teacherRec.duration());
            else if(mode==Mode::Compare)
                snprintf(title,sizeof(title),
                    "IMU Visualizer [COMPARE PAUSED %.1fs / %.1fs]  Tab=resume  [/]=scrub  R=exit",
                    teacherRec.currentTime(), teacherRec.duration());
            else if(mode==Mode::RecordTeacher)
                snprintf(title,sizeof(title),
                    "IMU Visualizer [● REC TEACHER %.1fs]  R=stop",
                    teacherRec.currentTime() + (now - lastTime));
            else if(mode==Mode::RecordStudent)
                snprintf(title,sizeof(title),
                    "IMU Visualizer [● REC STUDENT %.1fs]  R=stop",
                    studentRec.currentTime() + (now - lastTime));
            else
                snprintf(title,sizeof(title),
                    "IMU Visualizer - L_FA, R_FA, L_UA, R_UA, L_TH, L_SH, R_TH, R_SH"
                    "  |  T=teacher  S=student  F=compare  ESC=quit");
            glfwSetWindowTitle(window,title);
        }

        // ── Render ───────────────────────────────────────────────────────────
        if(mode==Mode::Compare){
            // Side-by-side: teacher left (blue tint), student right (red tint)
            int W,H; glfwGetFramebufferSize(window,&W,&H);
            int halfW=W/2;

            Pose tp = teacherRec.getPose();
            Pose sp = studentRec.getPose();

            // ── left half: teacher ───────────────────────────────────────────
            glViewport(0,0,halfW,H);
            glScissor(0,0,halfW,H);
            glEnable(GL_SCISSOR_TEST);

            glClearColor(0.04f,0.06f,0.12f,1.f);
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

            // set blue tint via a full-screen quad before the model
            glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
            glOrtho(0,1,0,1,-1,1);
            glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
            glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);
            glColor4f(0.0f,0.05f,0.18f,0.35f);
            glBegin(GL_QUADS);
            glVertex2f(0,0); glVertex2f(1,0); glVertex2f(1,1); glVertex2f(0,1);
            glEnd();
            glEnable(GL_DEPTH_TEST);
            glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix();
            glMatrixMode(GL_MODELVIEW);

            renderer.render(tp.lFA,tp.rFA,tp.lUA,tp.rUA,
                            tp.lTH,tp.rTH,tp.lSH,tp.rSH,
                            tp.hips,tp.chest,false);

            // "TEACHER" label
            glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
            glOrtho(0,halfW,0,H,-1,1);
            glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
            glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);
            glColor3f(0.55f,0.75f,1.0f);
            glRasterPos2f(halfW/2.0f-30.0f, H-30.0f);
            const char* tLabel="TEACHER";
            for(const char* c=tLabel;*c;++c) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*c);
            glEnable(GL_DEPTH_TEST);
            glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
            glDisable(GL_SCISSOR_TEST);

            // ── right half: student ──────────────────────────────────────────
            glViewport(halfW,0,W-halfW,H);
            glScissor(halfW,0,W-halfW,H);
            glEnable(GL_SCISSOR_TEST);

            glClearColor(0.12f,0.04f,0.04f,1.f);
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

            glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
            glOrtho(0,1,0,1,-1,1);
            glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
            glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);
            glColor4f(0.20f,0.03f,0.03f,0.35f);
            glBegin(GL_QUADS);
            glVertex2f(0,0); glVertex2f(1,0); glVertex2f(1,1); glVertex2f(0,1);
            glEnd();
            glEnable(GL_DEPTH_TEST);
            glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix();
            glMatrixMode(GL_MODELVIEW);

            renderer.render(sp.lFA,sp.rFA,sp.lUA,sp.rUA,
                            sp.lTH,sp.rTH,sp.lSH,sp.rSH,
                            sp.hips,sp.chest,false);

            // "STUDENT" label
            glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity();
            glOrtho(0,W-halfW,0,H,-1,1);
            glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
            glDisable(GL_DEPTH_TEST); glDisable(GL_LIGHTING);
            glColor3f(1.0f,0.5f,0.45f);
            glRasterPos2f((W-halfW)/2.0f-28.0f, H-30.0f);
            const char* sLabel="STUDENT";
            for(const char* c=sLabel;*c;++c) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,*c);
            glEnable(GL_DEPTH_TEST);
            glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
            glDisable(GL_SCISSOR_TEST);

            // Centre divider
            glViewport(0,0,W,H);
            glScissor(halfW-1,0,2,H);
            glEnable(GL_SCISSOR_TEST);
            glClearColor(0.50f,0.50f,0.55f,1.f);
            glClear(GL_COLOR_BUFFER_BIT);
            glDisable(GL_SCISSOR_TEST);
            glViewport(0,0,W,H);

        } else {
            // Normal live / recording render (full width, exact same as IMU_Reconstruction)
            renderer.render(live.lFA,live.rFA,live.lUA,live.rUA,
                            live.lTH,live.rTH,live.lSH,live.rSH,
                            live.hips,live.chest,false);
        }

        // ── CSV log (live mode only) ─────────────────────────────────────────
        if(mode==Mode::Live || mode==Mode::RecordTeacher || mode==Mode::RecordStudent){
            std::vector<SensorSample> samples = {
                {"L_FA",  live.lFA},  {"R_FA",  live.rFA},
                {"L_UA",  live.lUA},  {"R_UA",  live.rUA},
                {"L_TH",  live.lTH},  {"L_SH",  live.lSH},
                {"R_TH",  live.rTH},  {"R_SH",  live.rSH},
                {"HIPS",  live.hips}, {"CHEST", live.chest},
            };
            std::vector<bool> active = {
                sensorManager.isLFAActive(), sensorManager.isRFAActive(),
                sensorManager.isLUAActive(), sensorManager.isRUAActive(),
                sensorManager.isLTHActive(), sensorManager.isLSHActive(),
                sensorManager.isRTHActive(), sensorManager.isRSHActive(),
                sensorManager.isHipsActive(),sensorManager.isChestActive(),
            };
            csvLogger.log(samples,active);
        }

        glfwSwapBuffers(window);
    }

    csvLogger.close();
    receiver.detach();
    glfwTerminate();
    return 0;
}
