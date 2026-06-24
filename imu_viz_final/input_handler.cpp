#include "input_handler.h"
#include "renderer.h"
#include <iostream>
#include <sstream>

InputHandler::InputHandler(SensorManager& sensorManager, CsvLogger& csvLogger, Renderer& renderer)
    : sensorManager(sensorManager), csvLogger(csvLogger), renderer(renderer) {}

void InputHandler::handleKey(int key)
{
    if (key == GLFW_KEY_C) {
        sensorManager.calibrateLFA();
        csvLogger.markCalibration("L_FA");
    } else if (key == GLFW_KEY_V) {
        sensorManager.calibrateRFA();
        csvLogger.markCalibration("R_FA");
    } else if (key == GLFW_KEY_B) {
        sensorManager.calibrateLUA();
        csvLogger.markCalibration("L_UA");
    } else if (key == GLFW_KEY_N) {
        sensorManager.calibrateRUA();
        csvLogger.markCalibration("R_UA");
    } else if (key == GLFW_KEY_M) {
        sensorManager.toggleQuaternionConvention();
        csvLogger.markCalibration("MODE_" + std::to_string(sensorManager.getQuaternionMode() + 1));
    } else if (key == GLFW_KEY_P) {
        sensorManager.toggleVerticalOffset();
        csvLogger.markCalibration("VERTICAL_TOGGLE");
    } else if (key == GLFW_KEY_L) {
        sensorManager.togglePlacementGuideMode();
        csvLogger.markCalibration(sensorManager.isPlacementGuideMode() ? "PLACEMENT_GUIDE_ON" : "PLACEMENT_GUIDE_OFF");
    } else if (key == GLFW_KEY_Z) {
        sensorManager.calibrateLTH();
        csvLogger.markCalibration("L_TH");
    } else if (key == GLFW_KEY_X) {
        sensorManager.calibrateLSH();
        csvLogger.markCalibration("L_SH");
    } else if (key == GLFW_KEY_G) {
        sensorManager.calibrateRTH();
        csvLogger.markCalibration("R_TH");
    } else if (key == GLFW_KEY_H) {
        sensorManager.calibrateRSH();
        csvLogger.markCalibration("R_SH");
    } else if (key == GLFW_KEY_I) {
        sensorManager.calibrateHips();
        csvLogger.markCalibration("HIPS");
        renderer.resetTorsoNeutral();
    } else if (key == GLFW_KEY_O) {
        sensorManager.calibrateChest();
        csvLogger.markCalibration("CHEST");
        renderer.resetTorsoNeutral();
    } else if (key == GLFW_KEY_SPACE) {
        std::streambuf* oldBuf = std::cout.rdbuf();
        std::ostringstream temp;
        std::cout.rdbuf(temp.rdbuf());

        sensorManager.calibrateLFA();
        sensorManager.calibrateRFA();
        sensorManager.calibrateLUA();
        sensorManager.calibrateRUA();
        sensorManager.calibrateLTH();
        sensorManager.calibrateLSH();
        sensorManager.calibrateRTH();
        sensorManager.calibrateRSH();
        sensorManager.calibrateHips();
        sensorManager.calibrateChest();

        std::cout.rdbuf(oldBuf);
        std::cout << "Calibrated "
                  << "L_FA, R_FA, L_UA, R_UA, "
                  << "L_TH, L_SH, R_TH, R_SH, "
                  << "HIPS, CHEST\n";
        csvLogger.markCalibration("ALL");
        renderer.resetTorsoNeutral();
    }
}

void keyCallbackDispatcher(GLFWwindow* window, int key, int scancode,
                           int action, int mods)
{
    if (action != GLFW_PRESS) return;
    InputHandler* handler = static_cast<InputHandler*>(
        glfwGetWindowUserPointer(window));
    if (handler) handler->handleKey(key);
}