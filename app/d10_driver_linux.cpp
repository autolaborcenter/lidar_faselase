#include "d10_driver_linux.h"

#include "serial_linux.h"

#include <unistd.h>// close

#include <cmath>
#include <thread>


bool common_filter(faselase::point_t p) {
    constexpr static auto QUARTER = 5760 / 4;

    auto dir = p.dir();
    return dir < QUARTER || (5760 - QUARTER) <= dir;
}

bool front_filter(faselase::point_t p) {
    // std::atan2(0.14, 012 - 0.113) ≈ 1.52084;
    constexpr static auto LIMIT = static_cast<uint16_t>(5760 * 1.5 / (2 * M_PI));

    auto dir = p.dir();
    return dir < LIMIT || (5760 - LIMIT) <= dir;
}

bool back_filter(faselase::point_t p) {
    constexpr static auto QUARTER = 5760 / 4;

    auto dir = p.dir();
    return dir < QUARTER || (5760 - QUARTER) <= dir;
}

std::unordered_map<std::string, faselase::d10_t>
scan_lidars(std::mutex &mutex, std::condition_variable &signal) {
    std::unordered_map<std::string, faselase::d10_t> lidars;
    for (const auto &name : list_ports()) {
        auto fd = open_serial(name);
        if (fd < 0) continue;
        auto map_iterator = lidars.try_emplace(name).first;
        std::thread([&, name, map_iterator, fd] {
            auto &lidar = map_iterator->second;
            lidar.update_filter(common_filter);
            uint8_t buffer[256];
            uint8_t size = 0;
            do {
                auto n = read(fd, buffer + size, sizeof(buffer) - size);
                if (n <= 0) break;
                size = lidar.receive(buffer, size + n);
            } while (true);
            close(fd);
            {
                std::unique_lock<std::mutex> lock(mutex);
                lidars.erase(map_iterator);
            }
            signal.notify_all();
        }).detach();
    }

    return lidars;
}
