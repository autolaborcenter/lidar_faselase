#include "d10_driver_linux.h"

#include "serial_linux.h"

#include <unistd.h>// close

#include <iostream>
#include <thread>

std::unordered_map<std::string, faselase::d10_t>
scan_lidars(std::mutex &mutex, std::condition_variable &signal) {
    std::unordered_map<std::string, faselase::d10_t> lidars;
    for (const auto &name : list_ports()) {
        auto fd = open_serial(name);
        if (fd < 0) continue;
        std::cout << name << " is opened." << std::endl;

        auto map_iterator = lidars.try_emplace(name).first;
        std::thread([&, name, map_iterator, fd] {
            auto &lidar = map_iterator->second;
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
            std::cout << name << " is closed." << std::endl;
        }).detach();
    }

    return lidars;
}
