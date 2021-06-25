﻿#include "d10_t.hh"

#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <utility>

namespace faselase {
    class d10_t::implement_t {
        union frame_value_t {
            // 小端 = 低位在前 + 低字节在前
            struct {
                uint8_t
                    l0 : 1,
                    l1 : 7,
                    l2 : 4,
                    zero : 4;
            } len;
            struct {
                uint16_t
                    d0 : 7,
                    d1 : 6,
                    zero : 3;
            } dir;
            uint16_t value;
        };
        static_assert(sizeof(frame_value_t) == 2);

        struct point_t {
            uint16_t len, dir;
        };
        static_assert(sizeof(point_t) == 4);

        union frame_t {
            constexpr static uint32_t FRAME_BITS = 0x80'80'80'80;
            struct {
                // 小端 = 低位在前 + 低字节在前
                uint8_t
                    l2 : 4,
                    check_bits : 3,
                    bit0 : 1,

                    l1 : 7,
                    bit1 : 1,

                    d1 : 6,
                    l0 : 1,
                    bit2 : 1,

                    d0 : 7,
                    bit3 : 1;
            };
            uint32_t bits;
            uint8_t bytes[4];

            bool verify() const {
                if ((bits & FRAME_BITS) != 0x80'00'00'00) return false;
                return true;

                constexpr static uint8_t cbit[]{
                    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
                    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
                    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
                    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
                    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8};

                return ((cbit[bytes[1]] + cbit[bytes[2]] + cbit[bytes[3]]) & 0b111) == check_bits;
            }

            point_t data() const {
                return {
                    frame_value_t{.len{.l0 = l0, .l1 = l1, .l2 = l2}}.value,
                    frame_value_t{.dir{.d0 = d0, .d1 = d1}}.value,
                };
            }
        };

        static_assert(sizeof(frame_t) == 4);

        std::mutex _mutex;
        std::deque<point_t> _queue0{{1, 0}}, _queue1;

    public:
        size_t receive(void *buffer, size_t size) {
            auto ptr = reinterpret_cast<uint8_t *>(buffer);
            auto end = ptr + size;

            while (ptr + sizeof(frame_t) <= end) {
                auto frame = reinterpret_cast<frame_t *>(ptr);
                if (!frame->verify())
                    ++ptr;
                else {
                    auto point = frame->data();
                    if (point.len) {
                        if (point.dir > 5760) point.dir -= 5760;

                        std::lock_guard<decltype(_mutex)> lock(_mutex);

                        if (point.dir > _queue0.back().dir)
                            _queue0.push_back(point);
                        else {
                            _queue1 = std::move(_queue0);
                            _queue0 = std::deque<point_t>{point};
                        }

                        while (!_queue1.empty() && _queue1.front().dir <= point.dir)
                            _queue1.pop_front();
                    }
                    ptr += sizeof(frame_t);
                }
            }
            size = end - ptr;
            std::memmove(buffer, ptr, size);
            return size;
        }
    };

    d10_t::d10_t() : _implement(new implement_t) {}
    d10_t::d10_t(d10_t &&others) noexcept : _implement(std::exchange(others._implement, nullptr)) {}
    d10_t::~d10_t() { delete _implement; }

    size_t d10_t::receive(void *buffer, size_t size) { return _implement->receive(buffer, size); }
}// namespace faselase
