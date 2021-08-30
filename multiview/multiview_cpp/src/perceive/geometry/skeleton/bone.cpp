
#include "bone.hpp"

#include "skeleton-2d.hpp"

namespace perceive::skeleton
{
// --------------------------------------------------------------- keypoint name
//
KeypointName int_to_keypoint_name(int val) noexcept
{
   const auto r = KeypointName(val);
   switch(r) {
#define E(x) \
   case KeypointName::x: return KeypointName::x;
      E(NOSE);
      E(NOTCH);
      E(R_SHOULDER);
      E(R_ELBOW);
      E(R_WRIST);
      E(L_SHOULDER);
      E(L_ELBOW);
      E(L_WRIST);
      E(PELVIS);
      E(R_HIP);
      E(R_KNEE);
      E(R_ANKLE);
      E(L_HIP);
      E(L_KNEE);
      E(L_ANKLE);
      E(R_EYE);
      E(L_EYE);
      E(R_EAR);
      E(L_EAR);
      E(L_BIG_TOE);
      E(L_SMALL_TOE);
      E(L_HEAL);
      E(R_BIG_TOE);
      E(R_SMALL_TOE);
      E(R_HEAL);
   default: FATAL(format("enumerated keypointname out of range: {}", val));
#undef E
   }
   return r;
}

const char* str(const KeypointName r) noexcept
{
   switch(r) {
#define E(x) \
   case KeypointName::x: return #x;
      E(NOSE);
      E(NOTCH);
      E(R_SHOULDER);
      E(R_ELBOW);
      E(R_WRIST);
      E(L_SHOULDER);
      E(L_ELBOW);
      E(L_WRIST);
      E(PELVIS);
      E(R_HIP);
      E(R_KNEE);
      E(R_ANKLE);
      E(L_HIP);
      E(L_KNEE);
      E(L_ANKLE);
      E(R_EYE);
      E(L_EYE);
      E(R_EAR);
      E(L_EAR);
      E(L_BIG_TOE);
      E(L_SMALL_TOE);
      E(L_HEAL);
      E(R_BIG_TOE);
      E(R_SMALL_TOE);
      E(R_HEAL);
#undef E
   }
   FATAL(format("kBAM!, keypoint-name = {}", int(r)));
   return "<unknown>";
}

KeypointName to_keypoint_name(const string_view val) noexcept(false)
{
#define E(x) \
   if(val == #x) return KeypointName::x;
   E(NOSE);
   E(NOTCH);
   E(R_SHOULDER);
   E(R_ELBOW);
   E(R_WRIST);
   E(L_SHOULDER);
   E(L_ELBOW);
   E(L_WRIST);
   E(PELVIS);
   E(R_HIP);
   E(R_KNEE);
   E(R_ANKLE);
   E(L_HIP);
   E(L_KNEE);
   E(L_ANKLE);
   E(R_EYE);
   E(L_EYE);
   E(R_EAR);
   E(L_EAR);
   E(L_BIG_TOE);
   E(L_SMALL_TOE);
   E(L_HEAL);
   E(R_BIG_TOE);
   E(R_SMALL_TOE);
   E(R_HEAL);
#undef E
   throw std::runtime_error(
       format("could not convert integer '{}' to a KeypointName", val));
}

// ------------------------------------------------------------------- p2d bones
//
static constexpr const std::array<Bone, Bone::k_n_bones> get_p2d_bones_()
{
   constexpr uint32_t j1_2      = 0x00999900;
   constexpr uint32_t j2_3      = 0x00966400;
   constexpr uint32_t j3_4      = 0x00973200;
   constexpr uint32_t j1_5      = 0x00649700;
   constexpr uint32_t j5_6      = 0x00339900;
   constexpr uint32_t j6_7      = 0x00009800;
   constexpr uint32_t j0_1      = 0x00980032;
   constexpr uint32_t j1_8      = 0x00990000;
   constexpr uint32_t j0_15     = 0x00980065;
   constexpr uint32_t j15_17    = 0x00980098;
   constexpr uint32_t j0_16     = 0x00650098;
   constexpr uint32_t j16_18    = 0x00320098;
   constexpr uint32_t j8_9      = 0x00009632;
   constexpr uint32_t j8_12     = 0x00006496;
   constexpr uint32_t j9_10     = 0x00009664;
   constexpr uint32_t j10_11    = 0x00009999;
   constexpr uint32_t j12_13    = 0x00003299;
   constexpr uint32_t j13_14    = 0x00000098;
   constexpr uint32_t k_nothing = 0xFF000000;

   constexpr std::array<Bone, Bone::k_n_bones> bones_ = {
       {// Head
        {KeypointName::NOSE, KeypointName::L_EYE, j0_16, Bone::L_FACE, 0.3f},
        {KeypointName::NOSE, KeypointName::R_EYE, j0_15, Bone::R_FACE, 0.3f},
        {KeypointName::L_EYE, KeypointName::L_EAR, j16_18, Bone::L_SKULL, 0.4f},
        {KeypointName::R_EYE, KeypointName::R_EAR, j15_17, Bone::R_SKULL, 0.4f},

        // Neck
        {KeypointName::NOSE, KeypointName::NOTCH, j0_1, Bone::NECK, 1.0f},

        // Torso and Hips
        {KeypointName::NOTCH,
         KeypointName::L_SHOULDER,
         j1_5,
         Bone::L_COLLAR,
         1.25f},
        {KeypointName::NOTCH,
         KeypointName::R_SHOULDER,
         j1_2,
         Bone::R_COLLAR,
         1.25f},
        {KeypointName::NOTCH, KeypointName::PELVIS, j1_8, Bone::SPINE, 3.5f},
        {KeypointName::PELVIS,
         KeypointName::L_HIP,
         j8_12,
         Bone::L_PELVIS,
         1.25f},
        {KeypointName::PELVIS,
         KeypointName::R_HIP,
         j8_9,
         Bone::R_PELVIS,
         1.25f},

        // Arms
        {KeypointName::L_SHOULDER,
         KeypointName::L_ELBOW,
         j5_6,
         Bone::L_ARM_UPPER,
         2.0f},
        {KeypointName::L_ELBOW,
         KeypointName::L_WRIST,
         j6_7,
         Bone::L_ARM_LOWER,
         2.0f},
        {KeypointName::R_SHOULDER,
         KeypointName::R_ELBOW,
         j2_3,
         Bone::R_ARM_UPPER,
         2.0f},
        {KeypointName::R_ELBOW,
         KeypointName::R_WRIST,
         j3_4,
         Bone::R_ARM_LOWER,
         2.0f},

        // Legs
        {KeypointName::L_HIP,
         KeypointName::L_KNEE,
         j12_13,
         Bone::L_LEG_UPPER,
         3.0f},
        {KeypointName::L_KNEE,
         KeypointName::L_ANKLE,
         j13_14,
         Bone::L_LEG_LOWER,
         2.5f},
        {KeypointName::R_HIP,
         KeypointName::R_KNEE,
         j9_10,
         Bone::R_LEG_UPPER,
         3.0f},
        {KeypointName::R_KNEE,
         KeypointName::R_ANKLE,
         j10_11,
         Bone::R_LEG_LOWER,
         2.5f},

        // Feet
        {KeypointName::L_ANKLE,
         KeypointName::L_HEAL,
         k_nothing,
         Bone::L_FOOT,
         0.0f},
        {KeypointName::R_ANKLE,
         KeypointName::R_HEAL,
         k_nothing,
         Bone::R_FOOT,
         0.0f},
        {KeypointName::L_BIG_TOE,
         KeypointName::L_SMALL_TOE,
         k_nothing,
         Bone::L_TOES,
         0.0f},
        {KeypointName::R_BIG_TOE,
         KeypointName::R_SMALL_TOE,
         k_nothing,
         Bone::R_TOES,
         0.0f}}};

   return bones_;
}

const std::array<Bone, Bone::k_n_bones>& get_p2d_bones()
{
   static constexpr std::array<Bone, Bone::k_n_bones> bones_ = get_p2d_bones_();
   return bones_;
}

const Bone& get_bone(const Bone::Label label) noexcept
{
   const auto& bones = get_p2d_bones();
   Expects(size_t(label) < bones.size());
   Expects(bones[size_t(label)].label == label);
   return bones[size_t(label)];
}

bool has_bone(const Skeleton2D& p2d, const Bone& bone) noexcept
{
   const auto& kps = p2d.keypoints();
   return kps[size_t(bone.kp0)].is_valid() && kps[size_t(bone.kp1)].is_valid();
}

// ----------------------------------------------------------------- bone::label
//
Bone::Label int_to_bone_label(int val) noexcept
{
   const auto r = Bone::Label(val);
   switch(r) {
#define E(x) \
   case Bone::Label::x: return Bone::Label::x;
      E(L_FACE);
      E(R_FACE);
      E(L_SKULL);
      E(R_SKULL);
      E(NECK);
      E(L_COLLAR);
      E(R_COLLAR);
      E(SPINE);
      E(L_PELVIS);
      E(R_PELVIS);
      E(L_ARM_UPPER);
      E(L_ARM_LOWER);
      E(R_ARM_UPPER);
      E(R_ARM_LOWER);
      E(L_LEG_UPPER);
      E(L_LEG_LOWER);
      E(R_LEG_UPPER);
      E(R_LEG_LOWER);
      E(L_FOOT);
      E(R_FOOT);
      E(L_TOES);
      E(R_TOES);
   default: FATAL(format("enumerated Bone::Label out of range: {}", val));
#undef E
   }
   return r;
}

const char* str(const Bone::Label r) noexcept
{
   switch(r) {
#define E(x) \
   case Bone::Label::x: return #x;
      E(L_FACE);
      E(R_FACE);
      E(L_SKULL);
      E(R_SKULL);
      E(NECK);
      E(L_COLLAR);
      E(R_COLLAR);
      E(SPINE);
      E(L_PELVIS);
      E(R_PELVIS);
      E(L_ARM_UPPER);
      E(L_ARM_LOWER);
      E(R_ARM_UPPER);
      E(R_ARM_LOWER);
      E(L_LEG_UPPER);
      E(L_LEG_LOWER);
      E(R_LEG_UPPER);
      E(R_LEG_LOWER);
      E(L_FOOT);
      E(R_FOOT);
      E(L_TOES);
      E(R_TOES);
#undef E
   }
   FATAL("kBAM!");
   return "<unknown>";
}

Bone::Label to_bone_label(const string_view val) noexcept(false)
{
#define E(x) \
   if(val == #x) return Bone::Label::x;
   E(L_FACE);
   E(R_FACE);
   E(L_SKULL);
   E(R_SKULL);
   E(NECK);
   E(L_COLLAR);
   E(R_COLLAR);
   E(SPINE);
   E(L_PELVIS);
   E(R_PELVIS);
   E(L_ARM_UPPER);
   E(L_ARM_LOWER);
   E(R_ARM_UPPER);
   E(R_ARM_LOWER);
   E(L_LEG_UPPER);
   E(L_LEG_LOWER);
   E(R_LEG_UPPER);
   E(R_LEG_LOWER);
   E(L_FOOT);
   E(R_FOOT);
   E(L_TOES);
   E(R_TOES);
#undef E
   throw std::runtime_error(
       format("could not convert integer '{}' to a Bone::Label", val));
}

// ------------------------------------------------------- joints (joined bones)
//
static constexpr const std::array<Joint, Joint::k_n_joints> get_joints_()
{
   constexpr std::array<Joint, Joint::k_n_joints> joints_
       = {{{Bone::SPINE, Bone::NECK, Joint::BACK},
           {Bone::NECK, Bone::L_FACE, Joint::L_FACE},
           {Bone::NECK, Bone::R_FACE, Joint::R_FACE},
           {Bone::L_ARM_LOWER, Bone::L_ARM_UPPER, Joint::L_ELBOW},
           {Bone::R_ARM_LOWER, Bone::R_ARM_UPPER, Joint::R_ELBOW},
           {Bone::L_ARM_UPPER, Bone::L_COLLAR, Joint::L_SHOULDER},
           {Bone::R_ARM_UPPER, Bone::R_COLLAR, Joint::R_SHOULDER},
           {Bone::L_LEG_LOWER, Bone::L_LEG_UPPER, Joint::L_LEG},
           {Bone::R_LEG_LOWER, Bone::R_LEG_UPPER, Joint::R_LEG},
           {Bone::L_LEG_UPPER, Bone::SPINE, Joint::L_LEG_UPPER},
           {Bone::R_LEG_UPPER, Bone::SPINE, Joint::R_LEG_UPPER}}};
   return joints_;
}

const std::array<Joint, Joint::k_n_joints>& get_joints()
{
   static constexpr std::array<Joint, Joint::k_n_joints> joints_
       = get_joints_();
   return joints_;
}

const Joint& get_joint(const Joint::Label label) noexcept
{
   const auto& joints = get_joints();
   Expects(size_t(label) < joints.size());
   Expects(joints[size_t(label)].label == label);
   return joints[size_t(label)];
}

Joint::Label int_to_joint_label(int val) noexcept
{
   const auto r = Joint::Label(val);
   switch(r) {
#define E(x) \
   case Joint::Label::x: return Joint::Label::x;
      E(BACK);
      E(L_FACE);
      E(R_FACE);
      E(L_ELBOW);
      E(R_ELBOW);
      E(L_SHOULDER);
      E(R_SHOULDER);
      E(L_LEG);
      E(R_LEG);
      E(L_LEG_UPPER);
      E(R_LEG_UPPER);
   default: FATAL(format("enumerated Joint::Label out of range: {}", val));
#undef E
   }
   return r;
}

const char* str(const Joint::Label r) noexcept
{
   switch(r) {
#define E(x) \
   case Joint::Label::x: return #x;
      E(BACK);
      E(L_FACE);
      E(R_FACE);
      E(L_ELBOW);
      E(R_ELBOW);
      E(L_SHOULDER);
      E(R_SHOULDER);
      E(L_LEG);
      E(R_LEG);
      E(L_LEG_UPPER);
      E(R_LEG_UPPER);
#undef E
   }
   FATAL("kBAM!");
   return "<unknown>";
}

Joint::Label to_joint_label(const string_view val) noexcept(false)
{
#define E(x) \
   if(val == #x) return Joint::Label::x
   E(BACK);
   E(L_FACE);
   E(R_FACE);
   E(L_ELBOW);
   E(R_ELBOW);
   E(L_SHOULDER);
   E(R_SHOULDER);
   E(L_LEG);
   E(R_LEG);
   E(L_LEG_UPPER);
   E(R_LEG_UPPER);
#undef E
   throw std::runtime_error(
       format("could not convert integer '{}' to a Bone::Label", val));
}

// ------------------------------------------------------------- p2d proportions
//
Vector2f Skeleton2DProportions::get(const Skeleton2D& p2d) const noexcept
{
   return p2d.keypoints()[size_t(kp)].pos;
}

} // namespace perceive::skeleton
