#ifndef D10_T_HH
#define D10_T_HH

#include <cstddef>

namespace faselase {
    class d10_t {
        class implement_t;
        implement_t *_implement;

    public:
        d10_t();
        d10_t(d10_t const &) = delete;
        d10_t(d10_t &&) noexcept;
        ~d10_t();

        size_t receive(void *, size_t);
    };
}// namespace faselase

#endif// D10_T_HH
