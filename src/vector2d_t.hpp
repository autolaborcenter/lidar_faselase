#ifndef VECTOR2_T_HPP
#define VECTOR2_T_HPP

#include <cmath>

// 一个坐标
struct vector2d_t {
    int16_t x, y;
};

inline int16_t distance(vector2d_t a, vector2d_t b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

static_assert(sizeof(vector2d_t) == 4);

#endif// VECTOR2_T_HPP
