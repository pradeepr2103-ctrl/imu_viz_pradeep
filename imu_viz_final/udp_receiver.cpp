#include "udp_receiver.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstdio>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

void udpReceiver(SensorManager& sensorManager)
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    // Allow multiple sockets to bind to same port (needed for multicast)
    int reuse = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(5005);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return;
    }

    // Join multicast group 239.0.0.1
    ip_mreq mreq{};
    mreq.imr_multiaddr.s_addr = inet_addr("239.0.0.1");
    mreq.imr_interface.s_addr = INADDR_ANY;
    if (setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
        perror("multicast join failed (non-fatal, unicast still works)");

    std::cout << "Listening on UDP 5005 (multicast 239.0.0.1)\n";

    char buffer[256];

    while (true) {
        int len = recv(sockfd, buffer, sizeof(buffer) - 1, 0);
        if (len <= 0) continue;
        buffer[len] = '\0';

        // sscanf replaces stringstream + stof — no heap allocation
        char label[16];
        float w, x, y, z;
        if (sscanf(buffer, "%15[^,],%f,%f,%f,%f", label, &w, &x, &y, &z) != 5)
            continue;

        glm::quat q = glm::normalize(glm::quat(w, x, y, z));

        if      (!__builtin_strcmp(label, "L_FA"))  sensorManager.setLFAQuat(q);
        else if (!__builtin_strcmp(label, "R_FA"))  sensorManager.setRFAQuat(q);
        else if (!__builtin_strcmp(label, "L_UA"))  sensorManager.setLUAQuat(q);
        else if (!__builtin_strcmp(label, "R_UA"))  sensorManager.setRUAQuat(q);
        else if (!__builtin_strcmp(label, "L_TH"))  sensorManager.setLTHQuat(q);
        else if (!__builtin_strcmp(label, "L_SH"))  sensorManager.setLSHQuat(q);
        else if (!__builtin_strcmp(label, "R_TH"))  sensorManager.setRTHQuat(q);
        else if (!__builtin_strcmp(label, "R_SH"))  sensorManager.setRSHQuat(q);
        else if (!__builtin_strcmp(label, "HIPS"))  sensorManager.setHipsQuat(q);
        else if (!__builtin_strcmp(label, "CHEST")) sensorManager.setChestQuat(q);
    }
}