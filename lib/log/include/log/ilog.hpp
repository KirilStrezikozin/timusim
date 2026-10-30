/*
 * Copyright (C) 2025 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * TODO.
 */

#ifndef LOG_ILOG_HPP
#define LOG_ILOG_HPP

#include <iterator>

#include "log/fmt.hpp"
#include "span.hpp"

namespace logger {

class ILogger {
  public:
    using char_type = char;
    using size_type = std::size_t;

    /* Use `fmt::basic_memory_buffer` to avoid dynamic memory allocation for
     * small log messages. Their default is 500, use less here. A larger buffer
     * will be automatically allocated on heap if the formatted string exceeds
     * this size. */
    constexpr static size_type inline_buffer_size = 128;

    ILogger(const ILogger &)            = delete;
    ILogger(ILogger &&)                 = delete;
    ILogger &operator=(const ILogger &) = delete;
    ILogger &operator=(ILogger &&)      = delete;

    ILogger()                   = default;
    virtual ~ILogger() noexcept = default;

    template <typename... T> void debug(fmt::format_string<T...> fmt, T &&...args) {
        this->log_with_prefix_("DEBUG: ", fmt, std::forward<T>(args)...);
    }

    template <typename... T> void info(fmt::format_string<T...> fmt, T &&...args) {
        this->log_with_prefix_("INFO: ", fmt, std::forward<T>(args)...);
    }

    template <typename... T> void error(fmt::format_string<T...> fmt, T &&...args) {
        this->log_with_prefix_("ERROR: ", fmt, std::forward<T>(args)...);
    }

    template <typename... T> void warn(fmt::format_string<T...> fmt, T &&...args) {
        this->log_with_prefix_("WARN: ", fmt, std::forward<T>(args)...);
    }

    void clear_line(int width = 80) {
        fmt::basic_memory_buffer<char_type, inline_buffer_size> buf;

        fmt::format_to_n(std::back_inserter(buf), width + 1, "\r{:{}}", "", width);

        buf.push_back('\0');
        this->write_impl_({buf.data(), buf.size()});
    }

    template <typename... T> void println(fmt::format_string<T...> fmt, T &&...args) {
        fmt::basic_memory_buffer<char_type, inline_buffer_size> buf;

        fmt::format_to(std::back_inserter(buf), fmt, std::forward<T>(args)...);
        buf.push_back('\r'); /* For Arduino Serial monitor. */
        buf.push_back('\n');
        buf.push_back('\0');

        this->write_impl_({buf.data(), buf.size()});
    }

    template <typename... T> void print(fmt::format_string<T...> fmt, T &&...args) {
        fmt::basic_memory_buffer<char_type, inline_buffer_size> buf;

        fmt::format_to(std::back_inserter(buf), fmt, std::forward<T>(args)...);
        buf.push_back('\0');

        this->write_impl_({buf.data(), buf.size()});
    }

  protected:
    /* @brief Write the formatted string to the log.
     *
     * @note The string is guaranteed to be null-terminated. It might end with a newline.
     *
     * @param data The formatted string to write.
     */
    virtual void write_impl_(span<const char_type> data) const noexcept = 0;

    template <typename... T>
    void log_with_prefix_(const char *prefix, fmt::format_string<T...> fmt, T &&...args) {
        fmt::basic_memory_buffer<char_type, inline_buffer_size> buf;

        fmt::format_to(std::back_inserter(buf), "{}", prefix);
        fmt::format_to(std::back_inserter(buf), fmt, std::forward<T>(args)...);
        buf.push_back('\r'); /* For Arduino Serial monitor. */
        buf.push_back('\n');
        buf.push_back('\0');

        this->write_impl_({buf.data(), buf.size()});
    }
};

class NoopLogger final : public logger::ILogger {
  public:
    using Base = logger::ILogger;

  protected:
    void write_impl_([[maybe_unused]] span<const Base::char_type> data) const noexcept final {}
};

} /* namespace logger */

#endif /* LOG_ILOG_HPP */
