#include "d10_driver_linux.h"

#include <iostream>
#include <thread>

int main() {
    std::mutex mutex;
    std::condition_variable signal;

    auto ports = scan_lidars(mutex, signal);
    std::thread([&ports] {
        using clock = std::chrono::steady_clock;
        uint8_t buffer[1450];
        while (true) {
            auto now = clock::now();
            for (auto &[port, lidar] : ports) {
                auto n = lidar.snapshot(buffer, sizeof(buffer));
                std::cout << port << ": " << n << std::endl;
            }
            std::this_thread::sleep_until(now + std::chrono::milliseconds(50));
        }
    }).detach();

    std::string text;
    while (std::getline(std::cin, text)) {
    }

    return 0;
}
