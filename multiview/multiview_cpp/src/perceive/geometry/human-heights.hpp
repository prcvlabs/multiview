
#pragma once

namespace perceive::human
{
// Height ratios
constexpr real k_height_ratio_ankle    = 1.0 / 15.0;
constexpr real k_height_ratio_knee     = 4.0 / 15.0;
constexpr real k_height_ratio_hip      = 8.0 / 15.0;
constexpr real k_height_ratio_shoulder = 12.0 / 15.0;
constexpr real k_height_ratio_nose     = 13.5 / 15.0;
constexpr real k_height_ratio_eye      = 14.0 / 15.0;

constexpr real k_shoulder_width_adult_male   = 0.41;
constexpr real k_shoulder_width_adult_female = 0.36;

constexpr real k_adult_male_radius   = 0.5 * k_shoulder_width_adult_male;
constexpr real k_adult_female_radius = 0.5 * k_shoulder_width_adult_female;

constexpr real k_average_height = 0.5 * (64.0 + 70.0) * 0.0254;

enum class BodyKeypoint {
   L_ANKLE = 0,
   R_ANKLE,
   L_KNEE,
   R_KNEE,
   L_HIP,
   R_HIP,
   PELVIS,
   L_SHOULDER,
   R_SHOULDER,
   NOTCH,
   NOSE,
   L_EYE,
   R_EYE
};

// @param gender: { -1 => unknown, 0 => female, 1 => male }. Currently not used.
// @param age:    An integer, -1 for unknown. Currently not used.
real prob_height_between(int gender, int age, real low, real high) noexcept;
real height_z_score(int gender, int age, real height) noexcept;

real prob_height_between(real low, real high) noexcept;
real height_z_score(real height) noexcept;
inline real height_prob(real height, real interval) noexcept
{
   Expects(interval >= 0.0);
   return prob_height_between(height - 0.5 * interval, height + 0.5 * interval);
}

// A short two-year old
inline real min_height() noexcept { return 30 * 0.0254; }

// That's 2.15m
inline real max_height() noexcept { return 85 * 0.0254; }

} // namespace perceive::human
