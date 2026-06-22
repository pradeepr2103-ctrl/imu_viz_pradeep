#include "Renderer.h"
#include "Network.h"
#include "MotionRecorder.h"
#include <iostream>
#include <cstdlib>

int main() {
    std::system("mkdir -p recordings");

    mocap::Renderer renderer("visualizer/assets/models/human.glb");
    mocap::UdpReceiver udp;
    if (!udp.start())
        std::cerr << "UDP failed – running without sensor input" << std::endl;

    mocap::MotionRecorder teacherRec, studentRec;
    bool recordingTeacher = false, recordingStudent = false;
    bool playingComparison = false;

    // Live pose (all identity)
    std::array<mocap::Quat, 10> livePose;
    for (auto& q : livePose) q = {1.0f, 0.0f, 0.0f, 0.0f};

    while (!renderer.shouldClose()) {
        double now = glfwGetTime();

        // ── Keyboard handling ────────────────────────────────────────────
        GLFWwindow* win = renderer.window();

        if (glfwGetKey(win, GLFW_KEY_T) == GLFW_PRESS) {
            if (!recordingTeacher) {
                teacherRec.startRecording();
                recordingTeacher = true;
                std::cout << "Recording teacher...\n";
            }
        }
        else if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS) {
            if (!recordingStudent) {
                studentRec.startRecording();
                recordingStudent = true;
                std::cout << "Recording student...\n";
            }
        }
        else if (glfwGetKey(win, GLFW_KEY_R) == GLFW_PRESS) {
            // Stop & save
            if (recordingTeacher) {
                teacherRec.stopRecording("recordings/teacher.csv");
                recordingTeacher = false;
            }
            if (recordingStudent) {
                studentRec.stopRecording("recordings/student.csv");
                recordingStudent = false;
            }
        }
        else if (glfwGetKey(win, GLFW_KEY_P) == GLFW_PRESS) {
            // Load both recordings and play
            if (!playingComparison) {
                if (teacherRec.loadRecording("recordings/teacher.csv") &&
                    studentRec.loadRecording("recordings/student.csv")) {
                    teacherRec.play();
                    studentRec.play();
                    playingComparison = true;
                    std::cout << "Playing comparison\n";
                }
            }
        }
        else if (glfwGetKey(win, GLFW_KEY_SPACE) == GLFW_PRESS) {
            if (playingComparison) {
                if (teacherRec.isPlaying()) {
                    teacherRec.pause();
                    studentRec.pause();
                } else {
                    teacherRec.play();
                    studentRec.play();
                }
            }
        }
        else if (glfwGetKey(win, GLFW_KEY_LEFT) == GLFW_PRESS) {
            if (playingComparison) {
                teacherRec.seek(std::max(0.0, teacherRec.currentTime() - 1.0));
                studentRec.seek(std::max(0.0, studentRec.currentTime() - 1.0));
            }
        }
        else if (glfwGetKey(win, GLFW_KEY_RIGHT) == GLFW_PRESS) {
            if (playingComparison) {
                teacherRec.seek(teacherRec.currentTime() + 1.0);
                studentRec.seek(studentRec.currentTime() + 1.0);
            }
        }

        // ── Process UDP sensor data ─────────────────────────────────────
        auto samples = udp.getAllSamples();
        for (int i = 0; i < 10; ++i) {
            if (samples[i].everReceived) {
                livePose[i] = {samples[i].qw, samples[i].qx,
                               samples[i].qy, samples[i].qz};
            }
        }
        // Feed both recorders (they record the same live stream)
        for (int s = 0; s < 10; ++s) {
            teacherRec.onSensorUpdate(s, livePose[s]);
            studentRec.onSensorUpdate(s, livePose[s]);
        }

        // ── Update recorders ────────────────────────────────────────────
        teacherRec.update(now);
        studentRec.update(now);

        // ── Render ─────────────────────────────────────────────────────
        if (playingComparison) {
            renderer.renderCompare(teacherRec.getPose(), studentRec.getPose());
        } else {
            renderer.renderLive(livePose);
        }
    }

    udp.stop();
    return 0;
}