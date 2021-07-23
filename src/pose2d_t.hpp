#ifndef POSE2D_T_HPP
#define POSE2D_T_HPP

#include <cmath>

namespace mechdancer::geometry_2d {
    struct vector_t {
        int16_t x, y;
    };

    struct pose_t {
        vector_t pos;
        float dir;
    };

    class transformation_t {
        pose_t _pose;
        float _cos, _sin;

    public:
        explicit transformation_t(pose_t pose)
            : _pose(pose),
              _cos(std::cos(pose.dir)),
              _sin(std::sin(pose.dir)) {}

        auto operator()(vector_t v) const {
            return vector_t{
                static_cast<int16_t>(_cos * v.x - _sin * v.y + _pose.pos.x),
                static_cast<int16_t>(_sin * v.x + _cos * v.y + _pose.pos.y),
            };
        }

        auto operator()(pose_t p) const {
            return pose_t{
                operator()(p.pos),
                p.dir + _pose.dir,
            };
        }

        auto operator*(vector_t v) const { return operator()(v); }
        auto operator*(pose_t p) const { return operator()(p); }
    };

    static_assert(sizeof(vector_t) == 4);
    static_assert(sizeof(pose_t) == 8);

    inline int16_t distance(vector_t a, vector_t b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }
}// namespace mechdancer::geometry_2d

#endif// POSE2D_T_HPP
