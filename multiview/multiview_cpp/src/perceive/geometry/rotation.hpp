
#pragma once

#include "perceive/foundation.hpp"
#include "quaternion.hpp"
#include "vector-3.hpp"
#include "vector-4.hpp"
#include "vector.hpp"

namespace perceive
{
// Useful functions
bool is_rotation_matrix(real R[9]) noexcept;

// ---------------------------- Find rotations
// calc the rotation that transforms P1->W1 and P2->W2
// Expects: dot(P1, P2) == dot(W1, W2)
Quaternion calc_rotation(const Vector3& P1,
                         const Vector3& W1,
                         const Vector3& P2,
                         const Vector3& W2) noexcept;

std::pair<Vector3, Vector3> natural_basis(const Vector3& U) noexcept;

// Find the rotation from XYZ axis t
std::pair<Quaternion, Quaternion>
calc_rotation(const Vector3& C,     // a camera center
              const Vector3& ray_x, // ideal point for 'x'
              const Vector3& ray_y, // ideal point for 'y'
              const Vector3& X,     // 3D position of 'X'
              const Vector3& n      // Y = X + scalar * n
              ) noexcept;

Quaternion
look_at(const Vector3& C, const Vector3& at, const Vector3& up) noexcept;

// ---------------------------- Average rotations
Quaternion average_rotations(const Quaternion& a, const Quaternion& b) noexcept;
Quaternion average_rotations(const vector<Quaternion>& v) noexcept;

// ---------------------------- Transforming rotation representations

// quaternion -->
Vector4 quaternion_to_axis_angle(const Quaternion&) noexcept;
Vector3 quaternion_to_saa(const Quaternion&) noexcept;
Matrix3r quaternion_to_rot3x3(const Quaternion&) noexcept;
Vector3 quaternion_to_rodrigues(const Quaternion&) noexcept;

// axis-angle -->
Quaternion axis_angle_to_quaternion(const Vector4&) noexcept;
Vector3 axis_angle_to_saa(const Vector4&) noexcept;
Matrix3r axis_angle_to_rot3x3(const Vector4&) noexcept;
Vector3 axis_angle_to_rodrigues(const Vector4&) noexcept;

// saa -->
Quaternion saa_to_quaternion(const Vector3&) noexcept;
Vector4 saa_to_axis_angle(const Vector3&) noexcept;
Matrix3r saa_to_rot3x3(const Vector3&) noexcept;
Vector3 saa_to_rodrigues(const Vector3&) noexcept;

// Rodrigues -->
Quaternion rodrigues_to_quaternion(const Vector3&) noexcept;
Vector4 rodrigues_to_axis_angle(const Vector3&) noexcept;
Vector3 rodrigues_to_saa(const Vector3&) noexcept;
Matrix3r rodrigues_to_rot3x3(const Vector3&) noexcept;

// rot3x3 -->
Quaternion rot3x3_to_quaternion(const Matrix3r&) noexcept;
Vector4 rot3x3_to_axis_angle(const Matrix3r&) noexcept;
Vector3 rot3x3_to_saa(const Matrix3r&) noexcept;
Vector3 rot3x3_to_rodrigues(const Matrix3r&) noexcept;

// ---------------------------- Additional Rotations
// to/from axis-angle
void axis_angle_to_rotation3x3(const Vector4&, real R[9]) noexcept;
Vector4 rotation3x3_to_axis_angle(const real R[9]) noexcept;

// Additional quaternion to Matrix
Quaternion rotation3x3_to_quaternion(const real R[9]) noexcept;
void quaternion_to_rotation3x3(const Quaternion& q, real R[9]) noexcept;

void quaternion_to_Matrix3r(const Quaternion& q, Matrix3r& R) noexcept;
Matrix4r quaternion_to_rot4x4(const Quaternion& q) noexcept;

void quaternion_to_rotation4x4(const Quaternion& q,
                               real R[16],
                               bool row_maj) noexcept;

} // namespace perceive
