#pragma once
#include <array>
#include <atomic>
#include <thread>
#include <mutex>

namespace mocap {

struct LatestSample {
    float qw=1, qx=0, qy=0, qz=0;
    bool everReceived = false;
};

class UdpReceiver {
public:
    UdpReceiver();
    ~UdpReceiver();
    bool start();
    void stop();
    std::array<LatestSample, 10> getAllSamples() const;
private:
    std::atomic<bool> running_{false};
    std::thread recvThread_;
    mutable std::mutex mutex_;
    std::array<LatestSample, 10> samples_;
    void receiveLoop();
};

} // namespace mocap