
#pragma once

#include "perceive/geometry/human-heights.hpp"
#include "perceive/graphics/colour-set.hpp"

namespace perceive
{
struct Skeleton2D;
}

namespace perceive::skeleton
{
// --------------------------------------------------------------- p2d keypoints
//
enum KeypointName : int8_t {
   NOSE = 0,
   NOTCH,       // 1, ie. the NECK
   R_SHOULDER,  // 2
   R_ELBOW,     // 3
   R_WRIST,     // 4
   L_SHOULDER,  // 5
   L_ELBOW,     // 6
   L_WRIST,     // 7
   PELVIS,      // 8
   R_HIP,       // 9
   R_KNEE,      // 10
   R_ANKLE,     // 11
   L_HIP,       // 12
   L_KNEE,      // 13
   L_ANKLE,     // 14
   R_EYE,       // 15
   L_EYE,       // 16
   R_EAR,       // 17
   L_EAR,       // 18
   L_BIG_TOE,   // 19
   L_SMALL_TOE, // 20
   L_HEAL,      // 21
   R_BIG_TOE,   // 22
   R_SMALL_TOE, // 23
   R_HEAL       // 24
};

constexpr int k_n_keypoints = int(KeypointName::R_HEAL) + 1;

KeypointName int_to_keypoint_name(int) noexcept;
const char* str(const KeypointName) noexcept;
KeypointName to_keypoint_name(const string_view val) noexcept(false);
int keypoint_body_ind(const KeypointName) noexcept; // return '-1' if n/a

// ------------------------------------------------------------------- p2d bones
//
struct Bone
{
   using Joint = KeypointName;
   enum Label : int8_t {
      L_FACE = 0,
      R_FACE,
      L_SKULL,
      R_SKULL,
      NECK,
      L_COLLAR,
      R_COLLAR,
      SPINE,
      L_PELVIS,
      R_PELVIS,
      L_ARM_UPPER,
      L_ARM_LOWER,
      R_ARM_UPPER,
      R_ARM_LOWER,
      L_LEG_UPPER,
      L_LEG_LOWER,
      R_LEG_UPPER,
      R_LEG_LOWER,
      L_FOOT,
      R_FOOT,
      L_TOES,
      R_TOES
   };
   static constexpr int k_n_bones = 22;

   KeypointName kp0 = KeypointName::NOSE;
   KeypointName kp1 = KeypointName::NOSE;
   uint32_t kolour  = k_white;
   Label label      = NECK;

   // unit = sphere of back of cranium, a la Robert Hale
   // A person is 11.33 units high
   float proportion = fNAN;

   float length(float person_height) const noexcept
   {
      return person_height * proportion * adult_height_inv;
   }

   static constexpr float adult_height     = 11.33f; //
   static constexpr float adult_height_inv = 1.0f / adult_height;
};

const std::array<Bone, Bone::k_n_bones>& get_p2d_bones();
const Bone& get_bone(const Bone::Label label) noexcept;

bool has_bone(const Skeleton2D& p2d, const skeleton::Bone& bone) noexcept;

Bone::Label int_to_bone_label(int) noexcept;
const char* str(const Bone::Label) noexcept;
Bone::Label to_bone_label(const string_view val) noexcept(false);

// ------------------------------------------------------- joints (joined bones)
//
struct Joint
{
   enum Label : int8_t {
      BACK = 0,    // SPINE, NECK
      L_FACE,      // NECK, L_FACE,
      R_FACE,      // NECK, R_FACE
      L_ELBOW,     // L_ARM_LOWER, L_ARM_UPPER
      R_ELBOW,     // R_ARM_LOWER, R_ARM_UPPER
      L_SHOULDER,  // L_ARM_UPPER, L_COLLAR
      R_SHOULDER,  // R_ARM_UPPER, R_COLLAR
      L_LEG,       // L_LEG_LOWER, L_LEG_UPPER
      R_LEG,       // R_LEG_LOWER, R_LEG_UPPER
      L_LEG_UPPER, // L_LEG_UPPER, SPINE
      R_LEG_UPPER  // R_LEG_UPPER, SPINE
   };
   static constexpr int k_n_joints = 11;

   Bone::Label b0 = Bone::NECK;
   Bone::Label b1 = Bone::NECK;
   Label label    = BACK;
};

const std::array<Joint, Joint::k_n_joints>& get_joints();
const Joint& get_joint(const Joint::Label label) noexcept;

Joint::Label int_to_joint_label(int) noexcept;
const char* str(const Joint::Label) noexcept;
Joint::Label to_joint_label(const string_view val) noexcept(false);

// ------------------------------------------------------------- p2d proportions
//
struct Skeleton2DProportions
{
   human::BodyKeypoint type = {};
   real height_ratio        = dNAN; // [0..1] of the height
   real weight              = dNAN; // weight for this "keypoint"
   KeypointName kp          = {};   // The keypoint on a Skeleton2D

   // std::function<Vector2f(const Skeleton2D&)>
   //     get; // getter method from p2d skeleton
   Vector2f get(const Skeleton2D&) const noexcept;
};

constexpr std::array<Skeleton2DProportions, 13> k_p2d_proportions_ = {{
    {human::BodyKeypoint::L_ANKLE, human::k_height_ratio_ankle, 0.05, L_ANKLE},
    {human::BodyKeypoint::R_ANKLE, human::k_height_ratio_ankle, 0.05, R_ANKLE},
    {human::BodyKeypoint::L_KNEE, human::k_height_ratio_knee, 0.1, L_KNEE},
    {human::BodyKeypoint::R_KNEE, human::k_height_ratio_knee, 0.1, R_KNEE},
    {human::BodyKeypoint::L_HIP, human::k_height_ratio_hip, 1.0, L_HIP},
    {human::BodyKeypoint::R_HIP, human::k_height_ratio_hip, 1.0, R_HIP},
    {human::BodyKeypoint::PELVIS, human::k_height_ratio_hip, 0.9, PELVIS},
    {human::BodyKeypoint::L_SHOULDER,
     human::k_height_ratio_shoulder,
     1.1,
     L_SHOULDER},
    {human::BodyKeypoint::R_SHOULDER,
     human::k_height_ratio_shoulder,
     1.1,
     R_SHOULDER},
    {human::BodyKeypoint::NOTCH, human::k_height_ratio_shoulder, 1.32, NOTCH},
    {human::BodyKeypoint::NOSE, human::k_height_ratio_nose, 0.8, NOSE},
    {human::BodyKeypoint::L_EYE, human::k_height_ratio_eye, 0.5, L_EYE},
    {human::BodyKeypoint::R_EYE, human::k_height_ratio_eye, 0.5, R_EYE}
    //
}};

inline const auto& get_p2d_proportions() { return k_p2d_proportions_; }

} // namespace perceive::skeleton
