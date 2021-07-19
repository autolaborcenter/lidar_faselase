#ifndef D10_T_HH
#define D10_T_HH

#include "point_t.hpp"

namespace faselase {
    class d10_t {
        class implement_t;
        implement_t *_implement;

    public:
        d10_t();
        d10_t(d10_t const &) = delete;
        d10_t(d10_t &&) noexcept;
        ~d10_t();

        void update_filter(bool (*)(point_t));

        size_t receive(void *, size_t);
        size_t snapshot(void *, size_t) const;
        inline size_t snapshot(point_t *buffer, size_t size) const {
            return snapshot(reinterpret_cast<void *>(buffer), size * sizeof(point_t)) / sizeof(point_t);
        }
    };
}// namespace faselase

#endif// D10_T_HH
