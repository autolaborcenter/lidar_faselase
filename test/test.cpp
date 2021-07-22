#include "app/serial_linux.h"
#include "src/d10_t.hh"

#include <cstring>
#include <iostream>
#include <thread>

using namespace std::chrono_literals;

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
    using clock = std::chrono::steady_clock;

    faselase::d10_t lidar;
    launch_lidar("/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.2.1:1.0-port0", lidar);

    uint8_t buffer[1450];
    while (true) {
        const auto next = clock::now() + 100ms;
        auto n = lidar.snapshot(buffer, sizeof(buffer));
        std::cout << n / 3 << std::endl;
        std::this_thread::sleep_until(next);
    }
}
