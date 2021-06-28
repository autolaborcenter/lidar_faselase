#include "d10_driver_linux.h"

#include <arpa/inet.h>

#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

int main() {
    std::mutex mutex;
    std::condition_variable signal;

    sockaddr_in remote{.sin_family = AF_INET};

    auto ports = scan_lidars(mutex, signal);

    // 发送 udp
    std::thread([&ports, &remote] {
        using clock = std::chrono::steady_clock;
        auto udp = socket(AF_INET, SOCK_DGRAM, 0);
        uint8_t buffer[1450];
        buffer[0] = 255;
        while (true) {
            auto now = clock::now();
            for (auto &[port, lidar] : ports) {
                auto n = lidar.snapshot(buffer + 1, sizeof(buffer) - 1) + 1;
                if (remote.sin_addr.s_addr && remote.sin_port)
                    std::ignore = sendto(udp, buffer, n, MSG_WAITALL, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
            }
            std::this_thread::sleep_until(now + std::chrono::milliseconds(100));
        }
    }).detach();

    // 解析控制指令
    std::string text;
    while (std::getline(std::cin, text)) {
        std::stringstream builder(text);
        std::string address, port;
        std::getline(builder, address, ':');
        std::getline(builder, port);
        if (inet_pton(AF_INET, address.c_str(), &remote.sin_addr) <= 0)
            continue;
        try {
            auto p = std::stoi(port);
            if (p < 0 || 65535 < p)
                continue;
            remote.sin_port = htons(p);
        } catch (...) {
            continue;
        }
    }

    return 0;
}
