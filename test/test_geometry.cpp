#include "app/robot_outline.hpp"

#include <iostream>
#include <span>

using namespace autolabor::pm1;
using namespace mechdancer::geometry_2d;

int main() {
    {
        for (auto v : outline)
            std::cout << v.x << '\t' << v.y << std::endl;

        pose_t pose{{200, 0}, M_PI / 2};
        transformation_t trans(pose);

        auto transformed = trans(outline);
        std::cout << std::boolalpha
                  << check_inside(transformed, vector_t<>{0, 150}) << std::endl
                  << check_inside(transformed, vector_t<>{180, 50}) << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;
    {
        constexpr static auto square = {
            vector_t<>{-50, -50},
            vector_t<>{+50, -50},
            vector_t<>{+50, +50},
            vector_t<>{-50, +50},
        };
        auto transformed = transformation_t(pose_t{{}, M_PI_4})(square);
        for (auto v : transformed)
            std::cout << v.x << '\t' << v.y << std::endl;
        auto [min, max] = min_max(transformed);
        std::cout << "min = " << min.x << ' ' << min.y << ", max = " << max.x << ' ' << max.y << std::endl;
        for (auto v : enlarge(square, 25))
            std::cout << v.x << '\t' << v.y << std::endl;
    }

    return 0;
}
