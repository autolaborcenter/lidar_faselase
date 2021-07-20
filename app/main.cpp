#include "d10_driver_linux.h"

#include <arpa/inet.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <ranges>
#include <span>
#include <sstream>
#include <thread>

int main() {
    using namespace std::chrono_literals;

    std::cout << asinf(.06f / .34f) << std::endl;

    std::mutex mutex;
    std::condition_variable signal;
    std::atomic<sockaddr_in> remote({.sin_family = AF_INET});
    // 打开雷达
    auto ports = scan_lidars(mutex, signal);
    {
        auto size = ports.size();
        if (size == 0 || size > 2) return 1;
    }
    {// 确定前后
        faselase::point_t buffer[500];
        for (const auto _ : std::views::iota(1, 3)) {
            std::this_thread::sleep_for(120ms);// 保证更新一整帧
            std::unique_lock<decltype(mutex)> lock(mutex);
            if (ports.empty()) return 1;
            for (const auto &[port, lidar] : ports) {
                constexpr static auto FILTER = std::views::filter([](faselase::point_t p) {
                    // asinf(.06f / .34f) ≈ 0.1774f
                    constexpr static auto DIR = static_cast<uint16_t>(5760 * .18f / (2 * M_PI));
                    auto dir = p.dir();
                    return dir < DIR || (5760 - DIR) <= dir;
                });

                auto n = lidar.snapshot(buffer, sizeof(buffer));
                auto rudder = 0,// 认定为舵轮的点
                    all = 0;    // 角度内所有点
                for (auto p : std::span{buffer, n} | FILTER) {
                    ++all;
                    auto len = p.len();
                    if ((34 - 6) < len && len < 34) ++rudder;
                }
                std::cout << port << ": " << rudder << ", " << all << std::endl;
            }
        }
    }
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
        const auto next = clock::now() + 100ms;
        auto address = remote.load();
        if (address.sin_addr.s_addr && address.sin_port) {
            std::unique_lock<decltype(mutex)> lock(mutex);
            if (ports.empty()) break;
            for (const auto &[port, lidar] : ports) {
                auto n = lidar.snapshot(buffer + 1, sizeof(buffer) - 1) + 1;
                std::ignore = sendto(udp, buffer, n, MSG_WAITALL, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
            }
        }
        std::this_thread::sleep_until(next);
    }

    return 1;
}
