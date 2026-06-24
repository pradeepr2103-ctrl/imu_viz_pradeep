#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

struct SensorSample {
    std::string label;
    glm::quat   q;
};

class CsvLogger {
public:
    CsvLogger();
    ~CsvLogger();

    bool open();
    void log(const std::vector<SensorSample>& samples,
             const std::vector<bool>& active);
    void markCalibration(const std::string& sensorLabel);
    void close();

private:
    std::ofstream file;
    std::chrono::steady_clock::time_point programStartTime;
    int  frameCount    = 0;
    bool headerWritten = false;
    bool needsRewrite  = false;   // true when a new sensor joined, rewrite on next frame

    std::vector<bool>        everSeen;
    std::vector<std::string> lockedLabels;
    std::string              currentPath;

    static constexpr double kSettleSeconds = 5.0;
    static constexpr int    kFlushEveryN   = 60;

    std::string makeFilepath();
    void        writeHeader(const std::vector<SensorSample>& samples);
    void        rewriteWithNewHeader(const std::vector<SensorSample>& samples);
    static bool ensureDir(const std::string& dir);
};