#include "csv_logger.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>

CsvLogger::CsvLogger()  = default;
CsvLogger::~CsvLogger() { close(); }

bool CsvLogger::ensureDir(const std::string& dir)
{
    struct stat st{};
    if (stat(dir.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    return mkdir(dir.c_str(), 0755) == 0;
}

std::string CsvLogger::makeFilepath()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[64];
    std::strftime(buf, sizeof(buf), "CSV/%Y%m%d_%H%M%S.csv", &tm);
    return std::string(buf);
}

bool CsvLogger::open()
{
    if (!ensureDir("CSV")) {
        std::cerr << "[CsvLogger] Failed to create CSV/ directory\n";
        return false;
    }
    programStartTime = std::chrono::steady_clock::now();
    return true;
}

void CsvLogger::writeHeader(const std::vector<SensorSample>& samples)
{
    lockedLabels.clear();
    for (int i = 0; i < (int)everSeen.size(); ++i)
        if (everSeen[i]) lockedLabels.push_back(samples[i].label);

    file << "time";
    for (const auto& lbl : lockedLabels)
        file << ',' << lbl << "_w"
             << ',' << lbl << "_x"
             << ',' << lbl << "_y"
             << ',' << lbl << "_z";
    file << ",event\n";
    file.flush();
    headerWritten = true;
}

void CsvLogger::rewriteWithNewHeader(const std::vector<SensorSample>& samples)
{
    file.flush();
    file.close();

    std::string existingData;
    {
        std::ifstream reader(currentPath);
        if (reader.is_open()) {
            std::string oldHeader;
            std::getline(reader, oldHeader);
            std::ostringstream ss;
            ss << reader.rdbuf();
            existingData = ss.str();
        }
    }

    file.open(currentPath, std::ios::out | std::ios::trunc);
    if (!file.is_open()) {
        std::cerr << "[CsvLogger] Failed to reopen for rewrite\n";
        return;
    }
    writeHeader(samples);
    if (!existingData.empty())
        file << existingData;

    std::cout << "[CsvLogger] Header updated with new sensor\n";
}

void CsvLogger::log(const std::vector<SensorSample>& samples,
                    const std::vector<bool>& active)
{
    if (everSeen.size() < active.size())
        everSeen.resize(active.size(), false);

    // Detect newly seen sensors
    for (int i = 0; i < (int)active.size(); ++i) {
        if (active[i] && !everSeen[i]) {
            everSeen[i] = true;
            if (headerWritten)
                needsRewrite = true;
        }
    }

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - programStartTime).count();

    if (!headerWritten) {
        if (elapsed < kSettleSeconds) return;

        bool anySeen = false;
        for (bool s : everSeen) if (s) { anySeen = true; break; }
        if (!anySeen) return;

        currentPath = makeFilepath();
        file.open(currentPath, std::ios::out | std::ios::trunc);
        if (!file.is_open()) {
            std::cerr << "[CsvLogger] Failed to open: " << currentPath << "\n";
            return;
        }
        std::cout << "[CsvLogger] Logging to: " << currentPath << "\n";
        writeHeader(samples);
        return;
    }

    if (needsRewrite) {
        needsRewrite = false;
        rewriteWithNewHeader(samples);
        if (!file.is_open()) return;
    }

    if (!file.is_open()) return;

    file << std::fixed << std::setprecision(6) << elapsed;
    for (const auto& lbl : lockedLabels) {
        int idx = -1;
        for (int i = 0; i < (int)samples.size(); ++i)
            if (samples[i].label == lbl) { idx = i; break; }

        if (idx >= 0 && idx < (int)active.size() && active[idx]) {
            const glm::quat& q = samples[idx].q;
            file << ',' << q.w << ',' << q.x << ',' << q.y << ',' << q.z;
        } else {
            file << ",,,,";
        }
    }
    file << ",\n";

    if (++frameCount % kFlushEveryN == 0)
        file.flush();
}

void CsvLogger::markCalibration(const std::string& sensorLabel)
{
    if (!file.is_open()) return;

    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - programStartTime).count();

    file << std::fixed << std::setprecision(6) << elapsed;
    for (size_t i = 0; i < lockedLabels.size(); ++i)
        file << ",,,,";
    file << ",CALIBRATED_" << sensorLabel << "\n";
    file.flush();
}

void CsvLogger::close()
{
    if (file.is_open()) {
        file.flush();
        file.close();
        std::cout << "[CsvLogger] File closed.\n";
    }
}