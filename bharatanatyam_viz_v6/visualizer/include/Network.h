#pragma once
#include <array>
#include <atomic>
#include <thread>
#include <mutex>

namespace mocap {

struct SensorSample {
    float qw=1,qx=0,qy=0,qz=0;
    bool  received=false;
};

class UdpReceiver {
public:
    UdpReceiver();
    ~UdpReceiver();
    bool start(int port=5005);
    void stop();
    std::array<SensorSample,10> getAll() const;
private:
    std::atomic<bool>           running_{false};
    std::thread                 thread_;
    mutable std::mutex          mutex_;
    std::array<SensorSample,10> samples_;
};

} // namespace mocap
