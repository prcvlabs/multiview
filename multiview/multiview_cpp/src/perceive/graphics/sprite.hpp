
#pragma once

#include "perceive/geometry/vector.hpp"
#include <memory>

namespace perceive
{
class Sprite
{
 public:
   CUSTOM_NEW_DELETE(Sprite)

   using Face = Point3;

   struct Material
   {
      Vector4f diffuse;
      Vector4f specular;
      Vector4f ambient;
      Vector4f emission;
      float shininess = 0.0f;
   };

   struct Mesh
   {
      Material material;
      vector<Vector4> colors;
      vector<Vector3> norms;
      vector<Vector3> verts;
      vector<Face> faces;
   };

   struct Fragment
   {
      Fragment(int mid = 0, int fid = 0)
          : mesh_id(mid)
          , face_id(fid)
      {}
      int mesh_id{0};
      int face_id{0};
   };

 private:
   vector<Mesh> meshes_;
   Vector3 center_{0.0, 0.0, 0.0};
   Vector3 min_xyz_{0.0, 0.0, 0.0};
   Vector3 max_xyz_{0.0, 0.0, 0.0};

   vector<Fragment> fragments_;

   void update_minmax_xyz() noexcept;
   static Sprite load_helper_(const void* raw_ptr) noexcept(false);

 public:
   static Sprite load(const string& filename) noexcept(false);
   static Sprite load_from_memory(const string& data) noexcept(false);

   void set_meshes(const vector<Mesh> meshes);

   const vector<Mesh>& meshes() const noexcept { return meshes_; }
   const vector<Fragment>& fragments() const noexcept { return fragments_; }
   const Vector3& center() const noexcept { return center_; }
   const Vector3& min_xyz() const noexcept { return min_xyz_; }
   const Vector3& max_xyz() const noexcept { return max_xyz_; }

   string to_string() const noexcept;

   void apply_transform(const EuclideanTransform& et) noexcept;

   friend string str(const Sprite& sprite) noexcept
   {
      return sprite.to_string();
   }
};

void load(Sprite& sprite, const string& filename) noexcept(false);
void read(Sprite& sprite, const std::string& in) noexcept(false);

Sprite make_camera_sprite(const real size);

// @alpha  Force the sprite to use this alpha value.
//         NAN means use the loaded materials alpha values.
void render_gl(const Sprite& sprite, double alpha = dNAN) noexcept;

// This must be destroyed
unsigned make_model_displaylist(const Sprite& model);
void destroy_model_displaylist(unsigned id);

} // namespace perceive
