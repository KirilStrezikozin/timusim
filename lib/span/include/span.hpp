/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Span refers to a contiguous sequence of objects. Simple implementation
 * of C++20's `std::span` that only requires C++14.
 */

#ifndef SPAN_HPP
#define SPAN_HPP

#include <algorithm>
#include <cstddef>

template <typename T> class span {
  public:
    using size_type    = std::size_t;
    using element_type = T;

    constexpr span(T *data, size_type size) noexcept : data_(data), size_(size) {}
    template <std::size_t N> constexpr span(T (&arr)[N]) noexcept : data_(arr), size_(N) {}

    constexpr T *data() const noexcept { return data_; }
    [[nodiscard]] constexpr size_type size() const noexcept { return size_; }

    constexpr T &operator[](size_type index) const noexcept { return data_[index]; }
    constexpr T &front() const noexcept { return data_[0]; }
    constexpr T &back() const noexcept { return data_[size_ - 1]; }

    constexpr T *begin() const noexcept { return data_; }
    constexpr T *end() const noexcept { return data_ + size_; }

    constexpr span subspan(size_type offset, size_type count) const noexcept {
        if (offset > size_) { return span(nullptr, 0); /* Invalid subspan */ }
        count = std::min(count, size_ - offset);
        return span(data_ + offset, count);
    }

  private:
    T *data_;
    size_type size_;
};

#endif /* SPAN_HPP */
