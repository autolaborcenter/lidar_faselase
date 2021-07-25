﻿#ifndef POSE2D_T_HPP
#define POSE2D_T_HPP

#include <algorithm>
#include <cmath>
#include <ranges>
#include <vector>

#include <iostream>

namespace mechdancer::geometry_2d {
    template<class t = int16_t>
    struct vector_t {
        t x, y;

        vector_t &operator+=(vector_t others) {
            x += others.x;
            y += others.y;
            return *this;
        }
        vector_t &operator-=(vector_t others) {
            x -= others.x;
            y -= others.y;
            return *this;
        }
        vector_t operator+(vector_t others) const { return others += *this; }
        vector_t operator-(vector_t others) const { return vector_t{*this} -= others; }

        template<class r = t>
        r length() const {
            if constexpr (std::is_integral_v<r>)
                return std::lround(std::hypot(x, y));
            else
                return std::hypot(x, y);
        }

        template<class r = float>
        vector_t<r> normalize() const {
            auto len = length<r>();
            return {x / len, y / len};
        }
    };

    struct pose_t {
        vector_t<> pos;
        float dir;
    };

    static_assert(sizeof(pose_t) == 8);

    template<class t>
    inline auto dot(vector_t<t> a, vector_t<t> b) {
        return a.x * b.x + a.y * b.y;
    }

    template<class t>
    inline auto cross(vector_t<t> a, vector_t<t> b) {
        return a.x * b.y - a.y * b.x;
    }

    template<class t>
    inline auto distance(vector_t<t> a, vector_t<t> b) {
        return (a - b).length();
    }

    template<class t>
    concept polygon_t =
        std::ranges::sized_range<t> &&
        std::is_same_v<std::ranges::range_value_t<t>, vector_t<>>;

    // 判断未穿、上穿或下穿
    inline static int check_cross(vector_t<> v, vector_t<> v0, vector_t<> v1) {
        if (v0.y == v1.y) return 0;
        if (v0.y < v1.y) {// 判断线段向上还是向下
            // 射线穿过线段或线段下端点，且在起点在线段右侧 -> 线段上穿射线
            if (v0.y <= v.y && v.y < v1.y &&
                ((v0.x >= v.x && v1.x >= v.x) || cross(v1 - v0, v - v0) >= 0)) return +1;
        } else {
            // 射线穿过线段或线段下端点，且在起点在线段右侧 -> 线段下穿射线
            if (v1.y <= v.y && v.y < v0.y &&
                ((v0.x >= v.x && v1.x >= v.x) || cross(v0 - v1, v - v1) >= 0)) return -1;
        }
        return 0;
    }

    // 判断点在多边形内
    inline bool check_inside(polygon_t auto const &polygon, vector_t<> v) {
        auto size = std::ranges::size(polygon);
        if (size < 3) return false;

        auto it = std::ranges::begin(polygon),
             end = std::ranges::end(polygon);
        auto front = *it++,
             v0 = front;
        auto i = 0;
        while (it != end) {
            auto v1 = *it++;
            i += check_cross(v, v0, v1);
            v0 = v1;
        }
        return i + check_cross(v, v0, front);
    }

    class transformation_t {
        pose_t _pose;
        float _cos, _sin;

    public:
        explicit transformation_t(pose_t const &pose)
            : _pose(pose),
              _cos(std::cos(pose.dir)),
              _sin(std::sin(pose.dir)) {}

        auto operator()(vector_t<> v) const {
            return vector_t{
                static_cast<int16_t>(std::lround(_cos * v.x - _sin * v.y + _pose.pos.x)),
                static_cast<int16_t>(std::lround(_sin * v.x + _cos * v.y + _pose.pos.y)),
            };
        }

        auto operator()(pose_t p) const {
            return pose_t{
                operator()(p.pos),
                p.dir + _pose.dir,
            };
        }

        auto operator()(polygon_t auto const &polygon) const {
            using element_t = std::ranges::range_value_t<decltype(polygon)>;
            std::vector<element_t> result(std::ranges::size(polygon));
            std::ranges::copy(polygon | std::views::transform([this](auto v) { return operator()(v); }), result.begin());
            return result;
        }

        auto operator*(vector_t<> v) const { return operator()(v); }
        auto operator*(pose_t p) const { return operator()(p); }
        auto operator*(polygon_t auto const &p) const { return operator()(p); }
    };
}// namespace mechdancer::geometry_2d

#endif// POSE2D_T_HPP
