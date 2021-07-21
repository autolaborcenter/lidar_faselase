#include "serial_linux.h"
#include "src/d10_t.hh"

#include <arpa/inet.h>

#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <span>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

static bool front_filter(faselase::point_t p) {
    // std::atan2(0.14, 012 - 0.113) ≈ 1.52084;
    constexpr static auto LIMIT = static_cast<uint16_t>(5760 * 1.5 / (2 * M_PI));

    auto dir = p.dir();
    return dir < LIMIT || (5760 - LIMIT) <= dir;
}

static bool back_filter(faselase::point_t p) {
    constexpr static auto QUARTER = 5760 / 4;
    constexpr static auto DEG20 = 5760 * 25 / 360;

    auto dir = p.dir();
    return (DEG20 < dir && dir <= QUARTER) || ((5760 - QUARTER) < dir && dir <= (5760 - DEG20));
}

static void launch_lidar(const char *name, faselase::d10_t &lidar) {
    std::thread([dev = std::string(name), &lidar] {
        uint8_t buffer[256];
        uint8_t size = 0;
        while (true) {
            auto fd = open_serial(dev.c_str());
            do {
                auto n = read(fd, buffer + size, sizeof(buffer) - size);
                if (n <= 0) break;
                size = lidar.receive(buffer, size + n);
            } while (true);
            close(fd);
            std::this_thread::sleep_for(1s);
        }
    }).detach();
}

int main() {
    faselase::d10_t front, back;
    front.update_filter(front_filter);
    back.update_filter(back_filter);

    launch_lidar("/dev/serial/by-path/platform-70090000.xusb-usb-0:2.3.3.3:1.0-port0", front);
    launch_lidar("/dev/serial/by-path/platform-70090000.xusb-usb-0:2.3.3.4:1.0-port0", back);

    // 解析控制指令
    std::atomic<sockaddr_in> remote({.sin_family = AF_INET});
    std::thread([&remote] {
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
                if (0 <= p && p < 65536)
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
            auto offset = 1;
            auto m = front.snapshot(buffer + offset, sizeof(buffer) - offset);
            offset += m;
            auto n = back.snapshot(buffer + offset, sizeof(buffer) - offset);
            offset += n;
            *reinterpret_cast<uint16_t *>(buffer + offset) = m / 3;
            offset += sizeof(uint16_t);
            std::ignore = sendto(udp, buffer, offset, MSG_WAITALL, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
        }
        std::this_thread::sleep_until(next);
    }

    return 1;
}
