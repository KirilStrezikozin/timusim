/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 *
 * SPDX-License-Identifier: MIT
 *
 * Port of the eigen library for Arduino framework.
 */

#ifndef EIGEN_COMPAT_HPP
#define EIGEN_COMPAT_HPP

#pragma push_macro("F")
#pragma push_macro("B0")
#pragma push_macro("B1")
#pragma push_macro("B2")
#pragma push_macro("B3")
#pragma push_macro("B4")
#pragma push_macro("B5")
#pragma push_macro("C0")
#pragma push_macro("C1")
#pragma push_macro("C2")
#pragma push_macro("C3")
#pragma push_macro("D0")
#pragma push_macro("D1")
#pragma push_macro("D2")
#pragma push_macro("D3")
#pragma push_macro("abs")

#undef F
#undef B0
#undef B1
#undef B2
#undef B3
#undef B4
#undef B5
#undef C0
#undef C1
#undef C2
#undef C3
#undef D0
#undef D1
#undef D2
#undef D3
#undef abs

#include <Eigen/Core>
#include <Eigen/Dense>

#pragma pop_macro("abs")
#pragma pop_macro("D3")
#pragma pop_macro("D2")
#pragma pop_macro("D1")
#pragma pop_macro("D0")
#pragma pop_macro("C3")
#pragma pop_macro("C2")
#pragma pop_macro("C1")
#pragma pop_macro("C0")
#pragma pop_macro("B5")
#pragma pop_macro("B4")
#pragma pop_macro("B3")
#pragma pop_macro("B2")
#pragma pop_macro("B1")
#pragma pop_macro("B0")
#pragma pop_macro("F")

#endif /* EIGEN_COMPAT_HPP */
