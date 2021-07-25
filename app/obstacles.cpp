#include "obstacles.h"

#include "robot_outline.hpp"

#include <sstream>

namespace autolabor::pm1 {
    using namespace mechdancer::geometry_2d;

    std::pair<std::string, path_t> parse_path(std::string line) {
        std::stringstream builder(line);

        std::string id, temp;
        if (!(builder >> temp >> id)) return {};

        path_t path;
        path_t::value_type p;
        while (builder >> temp) {
            std::ranges::replace(temp, ',', ' ');
            if (std::stringstream(temp) >> p.pos.x >> p.pos.y >> p.dir)
                path.push_back(p);
            else
                return {};
        }
        return {std::move(id), std::move(path)};
    }

    auto enlarge(polygon_t auto polygon, uint16_t d) {
        const auto size = std::ranges::size(polygon);

        std::vector<vector_t<float>> dirs(size + 1);
        auto p = std::ranges::begin(polygon),
             end = std::ranges::end(polygon);
        auto front = *p;

        auto q = dirs.begin();
        ++q;

        // 计算归一化边向量
        while (true) {
            auto v0 = *p++;
            if (p == end) {
                dirs.front() = *q = (front - v0).normalize();
                break;
            }
            *q++ = (*p - v0).normalize();
        }

        q = dirs.begin();
        std::vector<vector_t<>> result(size);
        for (auto &r : result) {
            auto a = *q;
            auto b = *++q;
            auto diff = a - b;
            auto k = d / cross(a, b);
            r = {
                static_cast<int16_t>(std::lroundf(diff.x * k)),
                static_cast<int16_t>(std::lroundf(diff.y * k)),
            };
        }
        return result;
    }

    // 多边形外框
    auto min_max(polygon_t auto polygon) {
        auto ptr = std::ranges::begin(polygon),
             end = std::ranges::end(polygon);

        auto min = *ptr, max = *ptr;
        for (++ptr; ptr != end; ++ptr) {

            if (ptr->x > max.x)
                max.x = ptr->x;
            else if (ptr->x < min.x)
                min.x = ptr->x;

            if (ptr->y > max.y)
                max.y = ptr->y;
            else if (ptr->y < min.y)
                min.y = ptr->y;
        }

        return std::pair{min, max};
    }

    uint8_t check_collision(path_t const &path, std::vector<obstacles_t> const &obstacles) {
        if (path.empty()) return -1;
        for (auto i = 0; i < path.size(); ++i) {
            auto outline = enlarge(transformation_t(path[i])(autolabor::pm1::outline), i * 50);
            auto [min, max] = min_max(outline);
            for (const auto &group : obstacles)
                for (auto o : group)
                    if (min.x < o.x && o.x < max.x &&
                        min.y < o.y && o.y < max.y &&
                        check_inside(outline, o)) return i;
        }
        return -1;
    }
}// namespace autolabor::pm1
