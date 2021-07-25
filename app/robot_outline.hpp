#ifndef PM1_OUTLINE_HPP
#define PM1_OUTLINE_HPP

#include "src/pose2d_t.hpp"

#include <array>

namespace autolabor::pm1 {
    using vertex_t = mechdancer::geometry_2d::vector_t<int16_t>;

    constexpr auto half_outline = std::to_array({
        vertex_t{+250, +80},
        vertex_t{+120, +140},
        vertex_t{+100, +180},
        vertex_t{+100, +260},
        vertex_t{-100, +260},
        vertex_t{-100, +180},
        vertex_t{-250, +180},
        vertex_t{-470, +120},
    });

    template<size_t size>
    constexpr auto mirror_y(std::array<vertex_t, size> array) {
        std::array<vertex_t, 2 * size> result{};
        std::ranges::copy(array, result.begin());
        for (auto i = 0; i < size; ++i)
            result[2 * size - 1 - i] = {array[i].x, static_cast<int16_t>(-array[i].y)};
        return result;
    }

    constexpr auto outline = mirror_y(half_outline);
}// namespace autolabor::pm1

#endif// PM1_OUTLINE_HPP
