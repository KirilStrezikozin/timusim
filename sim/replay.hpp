/*
 * Copyright (C) 2026 Kiril V. Strezikozin
 * SPDX-License-Identifier: Apache-2.0
 *
 * A recorded or derived sensor-frame input: specific force (g) and rate (dps)
 * against time, read from a CSV and interpolated linearly between rows.
 */

#ifndef SIM_REPLAY_HPP
#define SIM_REPLAY_HPP

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <tl/expected.hpp>

namespace sim {

class Replay {
  public:
    using Vec3 = Eigen::Vector3d;

    /* Rows of t_s,fx_g,fy_g,fz_g,wx_dps,wy_dps,wz_dps after one header line, t_s increasing. */
    static tl::expected<Replay, std::string> load(const std::filesystem::path &path) {
        std::FILE *f = std::fopen(path.string().c_str(), "r");
        if (f == nullptr) { return tl::make_unexpected("cannot read " + path.string()); }
        Replay r;
        char line[512];
        bool header = true;
        while (std::fgets(line, sizeof(line), f) != nullptr) {
            if (header) {
                header = false;
                continue;
            }
            double v[7];
            if (std::sscanf(line, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) != 7) {
                continue;
            }
            if (!r.t_.empty() && v[0] <= r.t_.back()) {
                std::fclose(f);
                return tl::make_unexpected(path.string() + ": t_s must increase");
            }
            r.t_.push_back(v[0]);
            r.force_.emplace_back(v[1], v[2], v[3]);
            r.rate_.emplace_back(v[4], v[5], v[6]);
        }
        std::fclose(f);
        if (r.t_.size() < 2) { return tl::make_unexpected(path.string() + ": needs at least two rows"); }
        return r;
    }

    [[nodiscard]] double duration_s() const { return this->t_.back(); }

    /* Held at the first and last row outside the recorded span. */
    [[nodiscard]] std::pair<Vec3, Vec3> at(double t) const {
        if (t <= this->t_.front()) { return {this->force_.front(), this->rate_.front()}; }
        if (t >= this->t_.back()) { return {this->force_.back(), this->rate_.back()}; }
        const auto k   = static_cast<std::size_t>(std::ranges::upper_bound(this->t_, t) - this->t_.begin());
        const double a = (t - this->t_[k - 1]) / (this->t_[k] - this->t_[k - 1]);
        return {
            this->force_[k - 1] + a * (this->force_[k] - this->force_[k - 1]),
            this->rate_[k - 1] + a * (this->rate_[k] - this->rate_[k - 1]),
        };
    }

  private:
    std::vector<double> t_;
    std::vector<Vec3> force_;
    std::vector<Vec3> rate_;
};

} /* namespace sim */

#endif /* SIM_REPLAY_HPP */
