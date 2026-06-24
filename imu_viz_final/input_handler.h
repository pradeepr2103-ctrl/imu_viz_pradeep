#pragma once
#include <GLFW/glfw3.h>
#include "sensor_manager.h"
#include "csv_logger.h"

class Renderer;   // forward declaration — avoids pulling in renderer.h's GL headers here

class InputHandler {
public:
    InputHandler(SensorManager& sensorManager, CsvLogger& csvLogger, Renderer& renderer);
    void handleKey(int key);

private:
    SensorManager& sensorManager;
    CsvLogger&     csvLogger;
    Renderer&      renderer;
};

void keyCallbackDispatcher(GLFWwindow* window, int key, int scancode,
                           int action, int mods);