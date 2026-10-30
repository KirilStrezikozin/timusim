/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * TODO.
 */

#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <initializer_list>
#include <type_traits>

#include "log/ilog.hpp"
#include "tl/casts.hpp"

#pragma push_macro("abs")
#undef abs

/**
 * @brief 3D vector.
 */
template <typename T> union Vector3 {
    static_assert(std::is_arithmetic<T>::value, "Vector3 requires an arithmetic type.");

    T data[3];

    constexpr T &operator[](int index) { return data[index]; }
    constexpr const T &operator[](int index) const { return data[index]; }

    constexpr T &x() { return data[0]; }
    constexpr T &y() { return data[1]; }
    constexpr T &z() { return data[2]; }

    constexpr const T &x() const { return data[0]; }
    constexpr const T &y() const { return data[1]; }
    constexpr const T &z() const { return data[2]; }

    constexpr Vector3() : data{0, 0, 0} {}
    constexpr Vector3(T x, T y, T z) : data{x, y, z} {}

    Vector3(std::initializer_list<T> init) {
        auto it = init.begin();
        for (int i = 0; i < 3 && it != init.end(); ++i) {
            data[i] = *it++;
        }
    }

    constexpr Vector3(const Vector3<T> &other)     = default;
    constexpr Vector3(Vector3<T> &&other) noexcept = default;

    constexpr Vector3<T> &operator=(const Vector3<T> &other)     = default;
    constexpr Vector3<T> &operator=(Vector3<T> &&other) noexcept = default;

    constexpr Vector3<T> &operator=(const T &scalar) {
        x() = scalar;
        y() = scalar;
        z() = scalar;
        return *this;
    }

    ~Vector3() = default;

    constexpr T magnitude() const { return std::sqrt((x() * x()) + (y() * y()) + (z() * z())); }
    constexpr Vector3<T> normalized() const {
        T mag = magnitude();
        return (mag > static_cast<T>(0)) ? (*this / mag) : Vector3<T>{};
    }

    constexpr Vector3<T> abs() const {
        return Vector3(std::abs(x()), std::abs(y()), std::abs(z()));
    }

    constexpr Vector3<T> sign() const {
        return Vector3(
            (x() > static_cast<T>(0)) - (x() < static_cast<T>(0)),
            (y() > static_cast<T>(0)) - (y() < static_cast<T>(0)),
            (z() > static_cast<T>(0)) - (z() < static_cast<T>(0))
        );
    }

    constexpr Vector3<T> cross(const Vector3<T> &other) const {
        return Vector3(
            (y() * other.z()) - (z() * other.y()),
            (z() * other.x()) - (x() * other.z()),
            (x() * other.y()) - (y() * other.x())
        );
    }

    constexpr T dot(const Vector3<T> &other) const {
        return (x() * other.x()) + (y() * other.y()) + (z() * other.z());
    }

    constexpr Vector3<T> operator+(const Vector3<T> &other) const {
        return Vector3(x() + other.x(), y() + other.y(), z() + other.z());
    }

    constexpr Vector3<T> &operator+=(const Vector3<T> &other) {
        x() += other.x();
        y() += other.y();
        z() += other.z();
        return *this;
    }

    constexpr Vector3<T> operator-(const Vector3<T> &other) const {
        return Vector3(x() - other.x(), y() - other.y(), z() - other.z());
    }

    constexpr Vector3<T> &operator-=(const Vector3<T> &other) {
        x() -= other.x();
        y() -= other.y();
        z() -= other.z();
        return *this;
    }

    constexpr Vector3<T> operator*(const Vector3<T> &other) const {
        return Vector3(x() * other.x(), y() * other.y(), z() * other.z());
    }

    constexpr Vector3<T> &operator*=(const Vector3<T> &other) {
        x() *= other.x();
        y() *= other.y();
        z() *= other.z();
        return *this;
    }

    constexpr Vector3<T> operator/(const Vector3<T> &other) const {
        return Vector3(x() / other.x(), y() / other.y(), z() / other.z());
    }

    constexpr Vector3<T> &operator/=(const Vector3<T> &other) {
        x() /= other.x();
        y() /= other.y();
        z() /= other.z();
        return *this;
    }

    constexpr Vector3<T> operator*(T scalar) const {
        return Vector3(x() * scalar, y() * scalar, z() * scalar);
    }

    constexpr Vector3<T> &operator*=(T scalar) {
        x() *= scalar;
        y() *= scalar;
        z() *= scalar;
        return *this;
    }

    constexpr Vector3<T> operator/(T scalar) const {
        return Vector3(x() / scalar, y() / scalar, z() / scalar);
    }

    constexpr Vector3<T> &operator/=(T scalar) {
        x() /= scalar;
        y() /= scalar;
        z() /= scalar;
        return *this;
    }

    constexpr bool operator==(const Vector3<T> &other) const {
        return x() == other.x() && y() == other.y() && z() == other.z();
    }

    constexpr bool operator!=(const Vector3<T> &other) const { return !(*this == other); }

    constexpr T const *begin() const { return data; }
    constexpr T const *end() const { return data + (sizeof(data) / sizeof(T)); }
};

#pragma pop_macro("abs")

#endif /* MATH_UTILS_HPP */
