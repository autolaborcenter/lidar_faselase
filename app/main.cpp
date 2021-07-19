#include "d10_driver_linux.h"

#include <arpa/inet.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

int main() {
    using namespace std::chrono_literals;

    std::mutex mutex;
    std::condition_variable signal;
    std::atomic<sockaddr_in> remote({.sin_family = AF_INET});

    // 打开雷达
    auto ports = scan_lidars(mutex, signal);
    std::this_thread::sleep_for(150ms);// 保证得到一整帧

    // 解析控制指令
    std::thread([&ports, &remote] {
        std::string text;
        while (std::getline(std::cin, text)) {
            std::stringstream builder(text);
            std::string address, port;
            std::getline(builder, address, ':');
            std::getline(builder, port);
            in_addr temp;
            if (inet_pton(AF_INET, address.c_str(), &temp) <= 0) continue;
            try {
                auto p = std::stoi(port);
                if (0 < p && p < 65536)
                    remote.store(sockaddr_in{
                        .sin_family = AF_INET,
                        .sin_port = htons(p),
                        .sin_addr = temp,
                    });
            } catch (...) {
            }
        }
    }).detach();

    // 发送 udp
    using clock = std::chrono::steady_clock;
    auto udp = socket(AF_INET, SOCK_DGRAM, 0);
    uint8_t buffer[1450];
    buffer[0] = 255;
    while (true) {
        auto next = clock::now() + std::chrono::milliseconds(100);
        std::unique_lock<decltype(mutex)> lock(mutex);
        if (ports.empty()) return 0;
        for (const auto &[port, lidar] : ports) {
            auto address = remote.load();
            if (address.sin_addr.s_addr && address.sin_port) {
                auto n = lidar.snapshot(buffer + 1, sizeof(buffer) - 1) + 1;
                std::ignore = sendto(udp, buffer, n, MSG_WAITALL, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
            }
        }
        lock.unlock();
        std::this_thread::sleep_until(next);
    }
}
