
#pragma once

namespace perceive
{
enum class PoseAnnotation : int8_t {
   NONE  = 0,
   STAND = 1,
   WALK  = 2,
   SIT   = 3,
   LAY   = 4,
   PHONE = 5,
   OTHER = 6
};

const char* str(const PoseAnnotation&) noexcept;
char pose_to_char(const PoseAnnotation&) noexcept;

PoseAnnotation to_pose_annotation(const string_view) noexcept;
PoseAnnotation to_pose_annotation(const int enum_int) noexcept;
PoseAnnotation to_pose_annotation(const char) noexcept;

constexpr int n_pose_annotations() noexcept
{
   return int(PoseAnnotation::OTHER) + 1;
}

constexpr auto pose_icon_fnames = to_array<string_view>({"",
                                                         "pose-stand.png",
                                                         "pose-walk.png",
                                                         "pose-sit.png",
                                                         "pose-lay.png",
                                                         "pose-phone.png",
                                                         "pose-other.png"});

} // namespace perceive
