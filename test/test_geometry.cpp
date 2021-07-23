#include "src/pose2d_t.hpp"

#include <iostream>
#include <vector>

using namespace mechdancer::geometry_2d;

int main() {
    pose_t pose{{200, 0}, M_PI / 2};
    transformation_t trans(pose);
    std::vector<vector_t> outline{
        {0, 0},
        {100, 0},
        {50, 50},
        {0, 50},
    };

    auto transformed = trans(outline);
    for (auto v : transformed) std::cout << v.x << ' ' << v.y << std::endl;
    std::cout << std::boolalpha
              << check_inside(transformed, vector_t{0, 150}) << std::endl
              << check_inside(transformed, vector_t{180, 50}) << std::endl;
    return 0;
}
