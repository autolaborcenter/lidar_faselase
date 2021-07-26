#ifndef D10_T_HH
#define D10_T_HH

#include "geometry.hpp"
#include "point_t.hpp"

#include <vector>

namespace faselase {
    using v2d_t = mechdancer::geometry_2d::vector_t<int16_t>;

    class d10_t {
        class implement_t;
        implement_t *_implement;

    public:
        d10_t(v2d_t(point_t));
        d10_t(d10_t const &) = delete;
        d10_t(d10_t &&) noexcept;
        ~d10_t();

        void update_filter(bool (*)(point_t));

        // 从接收缓冲区解析
        size_t receive(void *, size_t);

        // 拷贝压缩编码的一帧
        size_t snapshot(void *, size_t) const;

        // 拷贝直角坐标的一帧
        std::vector<v2d_t> snapshot_map() const;
    };
}// namespace faselase

#endif// D10_T_HH
