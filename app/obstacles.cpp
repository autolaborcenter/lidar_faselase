#include "obstacles.h"

#include "robot_outline.hpp"

#include <algorithm>
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

    uint8_t check_collision(path_t const &path, std::vector<obstacles_t> const &obstacles) {
        if (path.empty()) return -1;
        constexpr static auto &pm1 = autolabor::pm1::outline;
        std::vector<vector_t<>> outline(pm1.size());
        for (auto i = 0; i < path.size(); ++i) {
            std::ranges::copy(pm1 | (transformation_t(path[i]) * (1 + i * .1f)).pipe(), outline.begin());
            auto [min, max] = min_max(outline);
            for (auto o : obstacles | std::views::join)
                if (min.x < o.x && o.x < max.x &&
                    min.y < o.y && o.y < max.y &&
                    check_inside(outline, o)) return i;
        }
        return -1;
    }
}// namespace autolabor::pm1
