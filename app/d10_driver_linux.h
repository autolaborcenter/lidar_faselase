#ifndef PM1_DRIVER_COMMON_H
#define PM1_DRIVER_COMMON_H

#include "src/d10_t.hh"

#include <condition_variable>
#include <string>
#include <unordered_map>

std::unordered_map<std::string, faselase::d10_t>
scan_lidars(std::mutex &mutex, std::condition_variable &signal);

#endif// PM1_DRIVER_COMMON_H
