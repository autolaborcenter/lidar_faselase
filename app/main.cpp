#include "d10_driver_linux.h"

int main() {
    std::mutex mutex;
    std::condition_variable signal;

    auto ports = scan_lidars(mutex, signal);

    std::this_thread::sleep_for(std::chrono::hours(24));
    return 0;
}
