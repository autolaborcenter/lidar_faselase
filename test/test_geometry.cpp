#include "app/robot_outline.hpp"

#include <iostream>

using namespace autolabor::pm1;
using namespace mechdancer::geometry_2d;

int main() {
    for (auto v : outline)
        std::cout << v.x << '\t' << v.y << std::endl;

    pose_t pose{{200, 0}, M_PI / 2};
    transformation_t trans(pose);

    auto transformed = trans(outline);
    std::cout << std::boolalpha
              << check_inside(transformed, vector_t<>{0, 150}) << std::endl
              << check_inside(transformed, vector_t<>{180, 50}) << std::endl;
    return 0;
}
