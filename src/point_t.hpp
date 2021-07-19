#ifndef POINT_T_HH
#define POINT_T_HH

#include <cstddef>
#include <cstdint>

namespace faselase {
    // 一个采样点
    class point_t {
        struct len_t {
            uint16_t value : 11, : 5;
        };
        struct dir_t {
            uint16_t : 3, value : 13;
        };

        uint8_t bytes[3];

    public:
        constexpr explicit point_t(uint16_t len = 0, uint16_t dir = 0)
            : bytes{
                  static_cast<uint8_t>(len),
                  static_cast<uint8_t>((dir << 3) | (len >> 8)),
                  static_cast<uint8_t>(dir >> 5),
              } {}

        void len(uint16_t value) { reinterpret_cast<len_t *>(bytes)->value = value; }
        uint16_t len() const { return reinterpret_cast<const len_t *>(bytes)->value; }

        void dir(uint16_t value) { reinterpret_cast<dir_t *>(bytes + 1)->value = value; }
        uint16_t dir() const { return reinterpret_cast<const dir_t *>(bytes + 1)->value; }
    };
}// namespace faselase

#endif// POINT_T_HH
