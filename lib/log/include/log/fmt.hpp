/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * Copyright (C) 2025 Riley Cornelius
 * Copyright (C) 2023 Clément Bonnet
 * Copyright (C) 2012 - present, Victor Zverovich and {fmt} contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Port of the {fmt} library for Arduino framework.
 */

#ifndef LOG_FMT_HPP
#define LOG_FMT_HPP

/* Significantly reduce binary size - https://vitaut.net/posts/2024/binary-size/ */
/* Disable locale support. */
#define FMT_USE_LOCALE    0
/* Only include formatting types when they are used (increase per call size,
 * but reduces library size). */
#define FMT_BUILTIN_TYPES 0
/* */

/* Use header-only version of the library. */
#define FMT_HEADER_ONLY

/* Backup conflicting macros. */
#pragma push_macro("F")
#pragma push_macro("B1")

/* Disable conflicting macros (Arduino framework). */
#undef B1
#undef F

/* Include the library. */
#include <fmt/format.h>
#include <fmt/ranges.h>

/* Restore conflicting macros. */
#pragma pop_macro("F")
#pragma pop_macro("B1")

#endif /* LOG_FMT_HPP */
