#include "app/obstacles.h"
#include "app/serial_linux.h"
#include "src/d10_t.hh"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;
using namespace autolabor::pm1;
using v2d_t = faselase::v2d_t;

static void launch_lidar(const char *name, faselase::d10_t &lidar) {
    std::thread([dev = std::string(name), &lidar] {
        uint8_t buffer[256];
        size_t size = 0;
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
    using clock = std::chrono::steady_clock;

    faselase::d10_t lidar([](auto p) {
        auto l = p.len() * 10;
        auto d = p.dir() * 2 * M_PI / 5760;
        return v2d_t{
            static_cast<int16_t>(std::cos(d) * l),
            static_cast<int16_t>(std::sin(d) * l),
        };
    });
    launch_lidar("/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.2.1:1.0-port0", lidar);

    // P 0.3|0.1 4,0,-3.24162e-05 16,0,-0.00292793 36,0,-0.00844421 62,0,-0.0159548 91,-1,-0.023911 119,-1,-0.0318672 147,-2,-0.0398233 175,-4,-0.0477795 203,-5,-0.0557357 231,-7,-0.0636919 259,-9,-0.0716481 287,-11,-0.0796042 315,-13,-0.0875604 343,-16,-0.0955166 371,-18,-0.103473

    // 解析控制指令
    std::string line;
    while (std::getline(std::cin, line))
        switch (line[0]) {
            // [P]ath -> [W]arn
            case 'P': {
                auto [id, path] = parse_path(line);
                if (!id.empty())
                    std::cout << "R " << id << ' ' << +check_collision(path, {lidar.snapshot_map()}) << std::endl;
            } break;
        }
}
