#include "obstacles.h"
#include "serial_linux.h"
#include "src/d10_t.hh"

#include <arpa/inet.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;
using v2d_t = faselase::v2d_t;

static bool front_filter(faselase::point_t p) {
    // std::atan2(0.14, 012 - 0.113) ≈ 1.52084;
    constexpr static auto LIMIT = static_cast<uint16_t>(5760 * 1.5 / (2 * M_PI));

    auto dir = p.dir();
    return dir < LIMIT || (5760 - LIMIT) <= dir;
}

static bool back_filter(faselase::point_t p) {
    constexpr static auto QUARTER = 5760 / 4;
    constexpr static auto DEG20 = 5760 * 30 / 360;

    auto dir = p.dir();
    return (DEG20 < dir && dir <= QUARTER) || ((5760 - QUARTER) < dir && dir <= (5760 - DEG20));
}

static void launch_lidar(const char *name, faselase::d10_t &lidar) {
    std::thread([dev = std::string(name), &lidar] {
        uint8_t buffer[256];
        uint8_t size = 0;
        while (true) {
            auto fd = open_serial(dev.c_str());
            if (fd >= 0)
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
    faselase::d10_t
        front([](faselase::point_t p) {
            auto l = p.len() * 10;
            auto d = p.dir() * 2 * M_PI / 5760;
            return v2d_t{
                static_cast<int16_t>(std::cos(d) * l + 118),
                static_cast<int16_t>(std::sin(d) * l),
            };
        }),
        back([](faselase::point_t p) {
            auto l = p.len() * -10;
            auto d = p.dir() * 2 * M_PI / 5760;
            return v2d_t{
                static_cast<int16_t>(std::cos(d) * l - 141),
                static_cast<int16_t>(std::sin(d) * l),
            };
        });
    std::atomic<sockaddr_in> remote({.sin_family = AF_INET});

    // 发送 udp
    std::thread([&] {
        using clock = std::chrono::steady_clock;

        front.update_filter(front_filter);
        back.update_filter(back_filter);

        launch_lidar("/dev/serial/by-path/platform-70090000.xusb-usb-0:2.4.1:1.0-port0", front);
        launch_lidar("/dev/serial/by-path/platform-70090000.xusb-usb-0:2.4.3.4:1.0-port0", back);

        auto udp = socket(AF_INET, SOCK_DGRAM, 0);
        uint8_t buffer[1450];
        buffer[0] = 255;
        while (true) {
            const auto next = clock::now() + 100ms;
            auto address = remote.load();
            if (address.sin_addr.s_addr && address.sin_port) {
                auto offset = 1;
                auto m = front.snapshot(buffer + offset, sizeof(buffer) - offset - sizeof(uint16_t));
                offset += m;
                auto n = back.snapshot(buffer + offset, sizeof(buffer) - offset - sizeof(uint16_t));
                offset += n;
                *reinterpret_cast<uint16_t *>(buffer + offset) = m / 3;
                offset += sizeof(uint16_t);
                std::ignore = sendto(udp, buffer, offset, MSG_WAITALL, reinterpret_cast<sockaddr *>(&remote), sizeof(remote));
            }
            std::this_thread::sleep_until(next);
        }
    }).detach();

    // 解析控制指令
    std::string line;
    while (std::getline(std::cin, line))
        switch (line[0]) {
            using namespace autolabor::pm1;
            // [P]ath -> [R]isk
            case 'P': {
                auto [id, path] = parse_path(line);
                if (!id.empty()) {
                    auto index = check_collision(path, {front.snapshot_map(), back.snapshot_map()});
                    std::cout << "R " << id << ' ' << +index << std::endl;
                }
            } break;
            default: {
                std::stringstream builder(line);
                std::string temp;
                in_addr address;
                uint16_t port;
                if (std::getline(builder, temp, ':') &&
                    inet_pton(AF_INET, temp.c_str(), &address) > 0 &&
                    builder >> port)
                    remote.store(sockaddr_in{
                        .sin_family = AF_INET,
                        .sin_port = htons(port),
                        .sin_addr = address,
                    });
            } break;
        }

    return 0;
}
