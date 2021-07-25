#ifndef OBSTACLES_H
#define OBSTACLES_H

#include "src/pose2d_t.hpp"

#include <string>
#include <vector>

namespace autolabor::pm1 {
    using path_t = std::vector<mechdancer::geometry_2d::pose_t>;
    using obstacles_t = std::vector<mechdancer::geometry_2d::vector_t<int16_t>>;

    // 从字符串中解析一条轨迹
    std::pair<std::string, path_t> parse_path(std::string line);

    // 根据轨迹和障碍物判断碰撞
    uint8_t check_collision(path_t const &path, std::vector<obstacles_t> const &obstacles);
}// namespace autolabor::pm1

#endif// OBSTACLES_H
