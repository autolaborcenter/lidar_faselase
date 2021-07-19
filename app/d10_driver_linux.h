#ifndef PM1_DRIVER_COMMON_H
#define PM1_DRIVER_COMMON_H

#include "src/d10_t.hh"

#include <condition_variable>
#include <string>
#include <unordered_map>

std::unordered_map<std::string, faselase::d10_t>
scan_lidars(std::mutex &mutex, std::condition_variable &signal);

bool common_filter(faselase::point_t);
bool front_filter(faselase::point_t);
bool back_filter(faselase::point_t);

#endif// PM1_DRIVER_COMMON_H
