#ifndef POSE2D_T_HPP
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

    template<class t, class u>
    inline auto dot(vector_t<t> a, vector_t<u> b) {
        return a.x * b.x + a.y * b.y;
    }

    template<class t, class u>
    inline auto cross(vector_t<t> a, vector_t<u> b) {
        return a.x * b.y - a.y * b.x;
    }

    template<class t, class u>
    inline auto distance(vector_t<t> a, vector_t<u> b) {
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
    // 作从点出发向 x 正方向的射线，线段上穿次数等于下穿次数则点在内部
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

    // 多边形扩张
    // 两条连续的边不能共线
    // 不能用于收缩
    inline auto enlarge(polygon_t auto polygon, uint16_t d) {
        const auto size = std::ranges::size(polygon);
        std::vector<vector_t<float>> dirs(size + 1);
        std::vector<vector_t<>> result(size);
        {// 计算归一化边向量
            auto p = std::ranges::begin(polygon),
                 end = std::ranges::end(polygon);
            auto front = *p;

            auto q = dirs.begin();
            ++q;

            while (true) {
                auto v0 = *p++;
                if (p == end) {
                    dirs.front() = *q = (front - v0).normalize();
                    break;
                }
                *q++ = (*p - v0).normalize();
            }
        }
        {
            // 偏移顶点
            auto it = dirs.begin();
            auto r = result.begin();
            for (const auto s : polygon) {// 此循环要求顺序执行以使用迭代器 `it`，故不宜使用 ranges
                auto a = *it;
                auto b = *++it;
                auto diff = a - b;
                auto k = d / cross(a, b);// 若 a b 共线，此处产生无穷大异常值
                *r++ = s + vector_t<>{
                               static_cast<int16_t>(std::lroundf(diff.x * k)),
                               static_cast<int16_t>(std::lroundf(diff.y * k)),
                           };
            }
        }
        return result;
    }

    // 多边形外框
    inline auto min_max(polygon_t auto polygon) {
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

        auto pipe() const {
            return std::views::transform([this](auto v) { return operator()(v); });
        }

        auto operator*(vector_t<> v) const { return operator()(v); }
        auto operator*(pose_t p) const { return operator()(p); }

        transformation_t &operator*=(float k) {
            _cos *= k;
            _sin *= k;
            return *this;
        }

        transformation_t operator*(float k) {
            auto copy = *this;
            return copy *= k;
        }
    };
}// namespace mechdancer::geometry_2d

#endif// POSE2D_T_HPP
