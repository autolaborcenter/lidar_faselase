#ifndef D10_T_HH
#define D10_T_HH

#include "point_t.hpp"
#include "vector2d_t.hpp"

#include <vector>

namespace faselase {
    class d10_t {
        class implement_t;
        implement_t *_implement;

    public:
        d10_t(vector2d_t(point_t));
        d10_t(d10_t const &) = delete;
        d10_t(d10_t &&) noexcept;
        ~d10_t();

        void update_filter(bool (*)(point_t));

        size_t receive(void *, size_t);

        // 拷贝压缩过的一帧
        size_t snapshot(void *, size_t) const;

        // 拷贝直角坐标的一帧
        std::vector<vector2d_t> snapshot_map() const;
    };
}// namespace faselase

#endif// D10_T_HH
