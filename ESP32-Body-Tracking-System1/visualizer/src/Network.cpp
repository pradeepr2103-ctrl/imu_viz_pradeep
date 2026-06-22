#include "Network.h"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace mocap {

UdpReceiver::UdpReceiver() {}
UdpReceiver::~UdpReceiver() { stop(); }

bool UdpReceiver::start() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "UDP socket creation failed" << std::endl;
        return false;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5005);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "UDP bind failed" << std::endl;
        close(sock);
        return false;
    }
    running_ = true;
    int fd = sock;
    recvThread_ = std::thread([this, fd]() {
        while (running_) {
            uint8_t buf[17];
            sockaddr_in from;
            socklen_t fromLen = sizeof(from);
            ssize_t n = recvfrom(fd, buf, sizeof(buf), 0, (sockaddr*)&from, &fromLen);
            if (n == 17) { // exact packet size
                int id = buf[0];
                if (id >= 0 && id < 10) {
                    float qw, qx, qy, qz;
                    memcpy(&qw, &buf[1], 4);
                    memcpy(&qx, &buf[5], 4);
                    memcpy(&qy, &buf[9], 4);
                    memcpy(&qz, &buf[13], 4);
                    std::lock_guard<std::mutex> lock(mutex_);
                    samples_[id].qw = qw; samples_[id].qx = qx;
                    samples_[id].qy = qy; samples_[id].qz = qz;
                    samples_[id].everReceived = true;
                }
            }
        }
        close(fd);
    });
    std::cout << "UDP receiver listening on port 5005" << std::endl;
    return true;
}

void UdpReceiver::stop() {
    running_ = false;
    if (recvThread_.joinable()) recvThread_.join();
}

std::array<LatestSample, 10> UdpReceiver::getAllSamples() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_;
}

} // namespace mocap