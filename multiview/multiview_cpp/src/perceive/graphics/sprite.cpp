

#include "sprite.hpp"
#include "stdinc.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#ifdef USING_OPENGL
#include "gl/gl-error.hpp"
#include "gl/gl-utils.hpp"

#include <GL/gl.h>
#include <GL/glu.h>
#endif

#define This Sprite

namespace perceive
{
// ------------------------------------------------------------- Apply Transform

void This::apply_transform(const EuclideanTransform& et) noexcept
{
   Matrix4r M       = make_transform_matrix(et);
   Matrix4r M_inv   = M.inverse();
   Matrix4r M_inv_t = M_inv.transpose();

   auto apply_mat = [](const Matrix4r& M, Vector3 v) -> Vector3 {
      Vector4r X = M * Vector4r(v.x, v.y, v.z, 1.0);
      X /= X(3);
      return Vector3(X(0), X(1), X(2));
   };

   for(auto& m : meshes_) {
      // Fix verts
      for(auto& v : m.verts) v = apply_mat(M, v);

      // Fix normals
      for(auto& n : m.norms) n = apply_mat(M_inv_t, n);
   }

   update_minmax_xyz();
}

void This::update_minmax_xyz() noexcept
{
   const auto max_val = std::numeric_limits<real>::max();
   const auto low_val = std::numeric_limits<real>::lowest();
   min_xyz_           = Vector3(max_val, max_val, max_val);
   max_xyz_           = Vector3(low_val, low_val, low_val);

   for(const auto& m : meshes_)
      for(const auto& X : m.verts)
         for(auto i = 0; i < 3; ++i) {
            if(X(i) < min_xyz_(i)) min_xyz_(i) = X(i);
            if(X(i) > max_xyz_(i)) max_xyz_(i) = X(i);
         }
   center_ = 0.5 * (min_xyz_ + max_xyz_);

   if(false) {
      INFO("print-xyz: ");
      cout << format("min = {}", str(min_xyz_)) << endl;
      cout << format("max = {}", str(max_xyz_)) << endl;
      cout << format("C   = {}", str(center_)) << endl;
   }
}

// ------------------------------------------------------------------- To String

string This::to_string() const noexcept
{
   return format(R"V0G0N(
Sprite:
   n-meshes:    {}
   n-faces:     {}
   center:      {}
   min-xyz:     {}
   max-xyz:     {}
{})V0G0N",
                 meshes().size(),
                 fragments_.size(),
                 str(center()),
                 str(min_xyz()),
                 str(max_xyz()),
                 "");
}

// ------------------------------------------------------------------------ Load

void load(Sprite& sprite, const string& filename) noexcept(false)
{
   sprite = Sprite::load(filename);
}

void read(Sprite& sprite, const std::string& in) noexcept(false)
{
   sprite = Sprite::load_from_memory(in);
}

// ------------------------------------------------------------------- Render Gl

void render_gl(const Sprite& sprite, double alphad) noexcept
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   static bool first = true;

   if(false) {
      first = false;

      glEnable(GL_LIGHTING);
      glEnable(GL_LIGHT0); // Uses default lighting parameters

      glEnable(GL_DEPTH_TEST); //
      glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
      glEnable(GL_NORMALIZE);
      glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
   }

   glShadeModel(GL_SMOOTH);
   glEnable(GL_POLYGON_SMOOTH);
   glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
   glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
   glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
   glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
   glEnable(GL_CULL_FACE);

   // -------------------------- Sort the fragments by distance from the camera
   static vector<Sprite::Fragment> frags;
   if(frags.size() != sprite.fragments().size())
      frags.resize(sprite.fragments().size());
   std::copy(
       cbegin(sprite.fragments()), cend(sprite.fragments()), begin(frags));

   Vector3 cam_center = extract_cam_center_from_current_model_view();

   auto face_dist = [&](const auto& mesh, const auto& face) -> real {
      Vector3 X;
      for(auto i = 0; i < 3; ++i) X += mesh.verts[size_t(face(i))];
      X /= 3;
      return (cam_center - X).quadrance();
   };

   std::sort(
       begin(frags), end(frags), [&](const auto& a, const auto& b) -> bool {
          const auto& m1 = sprite.meshes()[size_t(a.mesh_id)];
          const auto& m2 = sprite.meshes()[size_t(b.mesh_id)];
          auto& face1    = m1.faces[size_t(a.face_id)];
          auto& face2    = m2.faces[size_t(b.face_id)];
          return face_dist(m1, face1) > face_dist(m2, face2);
       });

   // ------------------------------------------------- Applies a material type

   auto apply_material = [alphad](int type, Vector4f val) {
      if(!std::isnan(alphad)) val[3] = float(alphad);
      glMaterialfv(GL_FRONT_AND_BACK, GLenum(type), val.ptr());
   };

   // ------------------------------------------------ Render a single fragment
   // Everytime we change mesh, we need to reapply materials
   int last_mesh_id     = -1;
   bool has_colors      = false;
   bool has_norms       = false;
   auto render_fragment = [&](const Sprite::Fragment& frag) {
      const auto& mesh = sprite.meshes()[size_t(frag.mesh_id)];

      if(frag.mesh_id != last_mesh_id) {
         glEnd();
         last_mesh_id  = frag.mesh_id;
         const auto& m = mesh.material;
         apply_material(GL_DIFFUSE, m.diffuse);
         apply_material(GL_SPECULAR, m.specular);
         apply_material(GL_AMBIENT, m.ambient);
         apply_material(GL_EMISSION, m.emission);

         glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 0.0f);
         if(m.shininess == 0.0f) {
            Vector4f spec{0.0f, 0.0f, 0.0f, 0.0f};
            glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec.ptr());
         }

         has_colors = mesh.colors.size() > 0;
         has_norms  = mesh.norms.size() > 0;

         if(has_norms)
            glEnable(GL_LIGHTING);
         else
            glDisable(GL_LIGHTING);
         glBegin(GL_TRIANGLES);
      }

      const auto& f = mesh.faces[size_t(frag.face_id)];
      const auto v0 = mesh.verts[size_t(f(0))];
      for(auto i = 0; i < 3; ++i) {
         if(has_colors) glColor4dv(mesh.colors[size_t(f(i))].ptr());
         if(has_norms) glNormal3dv(mesh.norms[size_t(f(i))].ptr());
         glVertex3dv(mesh.verts[size_t(f(i))].ptr());
      }
   };

   // ------------------------------------------------ Render all the fragments
   glBegin(GL_TRIANGLES);
   std::for_each(cbegin(frags), cend(frags), render_fragment);
   glEnd();
   glDisable(GL_LIGHTING);
#endif
}

void Sprite::set_meshes(const vector<Mesh> meshes)
{
   meshes_ = meshes;

   { // Create the mesh fragments...
      auto n_fragments
          = std::accumulate(cbegin(meshes_),
                            cend(meshes_),
                            0,
                            [](auto val, const auto& M) -> int {
                               return int(size_t(val) + M.faces.size());
                            });
      fragments_.clear();
      fragments_.reserve(size_t(n_fragments));
      for(auto i = 0u; i < meshes_.size(); ++i)
         for(auto j = 0u; j < meshes_[i].faces.size(); ++j)
            fragments_.emplace_back(i, j);
   }

   update_minmax_xyz();
}

// --------------------------------------------------------------- Load Material

static Sprite::Material load_material(const aiMaterial* mt)
{
   Sprite::Material out;

   aiColor4D diffuse;
   aiColor4D specular;
   aiColor4D ambient;
   aiColor4D emission;

   if(AI_SUCCESS == aiGetMaterialColor(mt, AI_MATKEY_COLOR_DIFFUSE, &diffuse)) {
      out.diffuse = Vector4f(diffuse[0], diffuse[1], diffuse[2], diffuse[3]);
   } else {
      out.diffuse = Vector4f(0.8f, 0.8f, 0.8f, 1.0f);
   }

   if(AI_SUCCESS
      == aiGetMaterialColor(mt, AI_MATKEY_COLOR_SPECULAR, &specular)) {
      out.specular
          = Vector4f(specular[0], specular[1], specular[2], specular[3]);
   } else {
      out.specular = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
   }

   if(AI_SUCCESS == aiGetMaterialColor(mt, AI_MATKEY_COLOR_AMBIENT, &ambient)) {
      out.ambient = Vector4f(ambient[0], ambient[1], ambient[2], ambient[3]);
   } else {
      out.ambient = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
   }

   if(AI_SUCCESS
      == aiGetMaterialColor(mt, AI_MATKEY_COLOR_EMISSIVE, &emission)) {
      out.emission
          = Vector4f(emission[0], emission[1], emission[2], emission[3]);
   } else {
      out.emission = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
   }

   float shininess  = NAN;
   float strength   = NAN;
   unsigned max_val = 1.0;
   if(AI_SUCCESS
      == aiGetMaterialFloatArray(
          mt, AI_MATKEY_SHININESS, &shininess, &max_val)) {
      max_val = 1;
      if(AI_SUCCESS
         == aiGetMaterialFloatArray(
             mt, AI_MATKEY_SHININESS_STRENGTH, &strength, &max_val)) {
         out.shininess = shininess * strength;
      } else {
         out.shininess = shininess;
      }
   } else {
      out.shininess = 0.0f;
   }

   return out;
}

// ------------------------------------------------------------------- Load Mesh

static Sprite::Mesh
load_mesh(const aiScene* sc, const Matrix4r& M, const aiMesh* mesh)
{
   Sprite::Mesh out;

   Matrix4r M_inv   = M.inverse();
   Matrix4r M_inv_t = M_inv.transpose();

   out.material = load_material(sc->mMaterials[mesh->mMaterialIndex]);

   const auto n_verts   = mesh->mNumVertices;
   const auto n_normals = mesh->mNormals == nullptr ? 0 : mesh->mNumVertices;
   const auto n_colours = mesh->mColors[0] == nullptr ? 0 : mesh->mNumVertices;
   const auto n_faces   = mesh->mNumFaces;

   auto place_vec3 = [&](const auto& M, auto& vec, const auto& X) {
      Vector4r Y(real(X.x), real(X.y), real(X.z), 1.0);
      Vector4r Z = M * Y;
      Z /= Z(3);
      vec.emplace_back(Z(0), Z(1), Z(2));
   };

   // Load vertices
   out.verts.reserve(n_verts);
   for(auto i = 0u; i < n_verts; ++i)
      place_vec3(M, out.verts, mesh->mVertices[i]);

   // Load normals
   out.norms.reserve(n_normals);
   for(auto i = 0u; i < n_normals; ++i)
      place_vec3(M_inv_t, out.norms, mesh->mNormals[i]);

   // Load colours
   out.colors.reserve(n_colours);
   for(auto i = 0u; i < n_colours; ++i) {
      auto c = mesh->mColors[0][i];
      out.colors.emplace_back(c[0], c[1], c[2], c[3]);
   }

   // Handle the faces
   for(auto t = 0u; t < n_faces; ++t) {
      auto face = &mesh->mFaces[t];
      switch(face->mNumIndices) {
      case 1:
         throw std::runtime_error("1-vertex face (i.e., point) "
                                  "in file");
         break; // point
      case 2:
         throw std::runtime_error("2-vertex face (i.e., line) "
                                  "in file");
         break; // line
      case 3:
         // Good.
         break; // triangle
      default:
         throw std::runtime_error(format("{}-vertex face (i.e., "
                                         "polygon) in file",
                                         mesh->mNumFaces));
         break; // polygon
      }

      out.faces.emplace_back(
          face->mIndices[0], face->mIndices[1], face->mIndices[2]);

      for(auto i = 0; i < 3; ++i)
         if(unsigned(out.faces.back()(i)) >= n_verts)
            throw std::runtime_error(format("face index out of range"));
   }

   return out;
}

// ----------------------------------------------------------------- Load Helper

Sprite This::load_helper_(const void* raw_ptr) noexcept(false)
{
   Sprite sprite;
   vector<Sprite::Mesh> meshes;
   const aiScene* scene = reinterpret_cast<const aiScene*>(raw_ptr);

   std::function<void(const Matrix4r&, const aiNode*)> recursive_load
       = [&](const Matrix4r& M0, const aiNode* nd) {
            // Update transform
            Matrix4r M = Matrix4r::Zero();
            {
               Matrix4r M1;
               const auto& m = nd->mTransformation;
               M1(0, 0)      = double(m.a1);
               M1(0, 1)      = double(m.a2);
               M1(0, 2)      = double(m.a3);
               M1(0, 3)      = double(m.a4);
               M1(1, 0)      = double(m.b1);
               M1(1, 1)      = double(m.b2);
               M1(1, 2)      = double(m.b3);
               M1(1, 3)      = double(m.b4);
               M1(2, 0)      = double(m.c1);
               M1(2, 1)      = double(m.c2);
               M1(2, 2)      = double(m.c3);
               M1(2, 3)      = double(m.c4);
               M1(3, 0)      = double(m.d1);
               M1(3, 1)      = double(m.d2);
               M1(3, 2)      = double(m.d3);
               M1(3, 3)      = double(m.d4);
               M             = M0 * M1;
            }

            // Update transform
            meshes.reserve(nd->mNumMeshes);
            for(auto n = 0u; n < nd->mNumMeshes; ++n)
               meshes.emplace_back(
                   load_mesh(scene, M, scene->mMeshes[nd->mMeshes[n]]));

            // Apply to the children
            for(auto n = 0u; n < nd->mNumChildren; ++n)
               recursive_load(M, nd->mChildren[n]);
         };

   Matrix4r M = Matrix4r::Identity();
   recursive_load(M, scene->mRootNode);

   sprite.set_meshes(meshes);
   return sprite;
}

// ------------------------------------------------------------------------ Load

Sprite This::load(const string& filename) noexcept(false)
{
   Assimp::Importer importer;
   // The 'importer' owns 'scene', and release memory in its destructor
   const aiScene* scene = importer.ReadFile(
       filename,
       aiProcess_CalcTangentSpace | aiProcess_Triangulate
           | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

   if(scene == nullptr)
      throw std::runtime_error(format("assimp failed to load '{}': {}",
                                      filename,
                                      importer.GetErrorString()));

   return load_helper_(scene);
}

// ------------------------------------------------------------ Load from Memory

Sprite This::load_from_memory(const string& data) noexcept(false)
{
   Assimp::Importer importer;
   // The 'importer' owns 'scene', and release memory in its destructor

   const aiScene* scene = importer.ReadFileFromMemory(
       &data[0],
       data.size(),
       aiProcess_CalcTangentSpace | aiProcess_Triangulate
           | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType);

   if(scene == nullptr)
      throw std::runtime_error(
          format("assimp failed to load: {}", importer.GetErrorString()));

   return load_helper_(scene);
}

// -------------------------------------------------------------------------- //
// --                         Make Camera Sprite                           -- //
// -------------------------------------------------------------------------- //

Sprite make_camera_sprite(const real size)
{
   vector<Sprite::Mesh> meshes(5);

   const auto cylindar_sides = 20;
   const auto radius         = size * 2.0 / 3.0;

   auto make_normals = [](auto& m) {
      // Calculate the normals
      m.norms.resize(m.verts.size());
      std::fill(begin(m.norms), end(m.norms), Vector3(0, 0, 0));
      for(const auto& f : m.faces) {
         Plane p3(m.verts[f(0)], m.verts[f(1)], m.verts[f(2)]);
         auto n = p3.xyz().normalized();
         for(auto i = 0; i < 3; ++i) m.norms[f(i)] += n;
      }

      for(auto& n : m.norms) n.normalise();
   };

   { // The body of the camera
      auto& m              = meshes[0];
      m.material.diffuse   = Vector4f(0.7f, 0.7f, 0.7f, 1.0f);
      m.material.specular  = Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
      m.material.ambient   = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
      m.material.emission  = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
      m.material.shininess = 0.0f;

      auto push_face = [&](int face_n) {
         const auto s = size;
         Vector3 c, dx, dy;

         switch(face_n) {
         case 0:
            c  = Vector3(0, 0, -s);
            dx = Vector3(0, s, 0);
            dy = Vector3(s, 0, 0);
            break;
         case 1:
            c  = Vector3(0, 0, s);
            dx = Vector3(-s, 0, 0);
            dy = Vector3(0, -s, 0);
            break;
         case 2:
            c  = Vector3(0, -s, 0);
            dx = Vector3(s, 0, 0);
            dy = Vector3(0, 0, s);
            break;
         case 3:
            c  = Vector3(0, s, 0);
            dx = Vector3(0, 0, s);
            dy = Vector3(s, 0, 0);
            break;
         case 4:
            c  = Vector3(s, 0, 0);
            dx = Vector3(0, s, 0);
            dy = Vector3(0, 0, s);
            break;
         case 5:
            c  = Vector3(-s, 0, 0);
            dx = Vector3(0, 0, s);
            dy = Vector3(0, s, 0);
            break;
         };

         int ref = int(m.verts.size());
         m.verts.emplace_back(c + dx + dy); // 0
         m.verts.emplace_back(c + dx - dy); // 1
         m.verts.emplace_back(c - dx + dy); // 2
         m.verts.emplace_back(c - dx - dy); // 3

         m.faces.emplace_back(1 + ref, 0 + ref, 2 + ref);
         m.faces.emplace_back(2 + ref, 3 + ref, 1 + ref);

         const auto& f = m.faces.back();
         Plane p3(m.verts[size_t(f(0))],
                  m.verts[size_t(f(1))],
                  m.verts[size_t(f(2))]);
         auto n = p3.xyz().normalized();
         for(auto i = 0; i < 4; ++i) m.norms.push_back(n);
      };

      // There are six faces to push
      push_face(0);
      push_face(1);
      push_face(2);
      push_face(3);
      push_face(4);
      push_face(5);
   }

   { // Lens cylindar of the camera
      auto& m              = meshes[1];
      m.material.diffuse   = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
      m.material.specular  = Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
      m.material.ambient   = Vector4f(0.4f, 0.2f, 0.2f, 1.0f);
      m.material.emission  = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
      m.material.shininess = 0.0f;

      const auto cylindar_sides = 20;
      const auto slice_theta    = 2.0 * M_PI / real(cylindar_sides);
      const auto C1             = Vector3(0, 0, 0);
      const auto C2             = Vector3(0, 0, size * 2.0);
      const auto radius         = size * 2.0 / 3.0;
      const auto dx             = Vector3(1, 0, 0);
      const auto dy             = Vector3(0, 1, 0);

      for(auto i = 0; i < cylindar_sides; ++i) {
         const auto theta1 = (i + 0) * slice_theta;
         const auto theta2 = (i + 1) * slice_theta;
         const auto A = C1 + radius * (cos(theta1) * dx + sin(theta1) * dy);
         const auto B = C1 + radius * (cos(theta2) * dx + sin(theta2) * dy);
         const auto C = C2 + radius * (cos(theta1) * dx + sin(theta1) * dy);
         const auto D = C2 + radius * (cos(theta2) * dx + sin(theta2) * dy);

         int ref = int(m.verts.size());
         m.verts.emplace_back(B);
         m.verts.emplace_back(A);
         m.verts.emplace_back(0.5 * (B + D));
         m.verts.emplace_back(0.5 * (A + C));
         m.verts.emplace_back(D);
         m.verts.emplace_back(C);

         auto a_norm = A.normalized();
         auto b_norm = B.normalized();
         m.norms.emplace_back(b_norm);
         m.norms.emplace_back(a_norm);
         m.norms.emplace_back(b_norm);
         m.norms.emplace_back(a_norm);
         m.norms.emplace_back(b_norm);
         m.norms.emplace_back(a_norm);

         // Two-sided
         m.faces.emplace_back(ref + 0, ref + 2, ref + 1);
         m.faces.emplace_back(ref + 0, ref + 2, ref + 3);
         m.faces.emplace_back(ref + 2, ref + 4, ref + 3);
         m.faces.emplace_back(ref + 3, ref + 4, ref + 5);

         // The other side
         m.faces.emplace_back(ref + 0, ref + 1, ref + 2);
         m.faces.emplace_back(ref + 0, ref + 3, ref + 2);
         m.faces.emplace_back(ref + 2, ref + 3, ref + 4);
         m.faces.emplace_back(ref + 3, ref + 5, ref + 4);
      }
   }

   { // Lens (back) of the camera
      auto& m              = meshes[2];
      m.material.diffuse   = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
      m.material.specular  = Vector4f(0.4f, 0.2f, 0.4f, 1.0f);
      m.material.ambient   = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
      m.material.emission  = Vector4f(0.0f, 0.0f, 0.0f, 1.0f);
      m.material.shininess = 0.1f;

      const auto slice_theta = 2.0 * M_PI / real(cylindar_sides);
      const auto C           = Vector3(0, 0, 1.2 * size);
      const auto dx          = Vector3(1, 0, 0);
      const auto dy          = Vector3(0, 1, 0);

      for(auto i = 0; i < cylindar_sides; ++i) {
         const auto theta1 = (i + 0) * slice_theta;
         const auto theta2 = (i + 1) * slice_theta;
         const auto A      = C + radius * (cos(theta1) * dx + sin(theta1) * dy);
         const auto B      = C + radius * (cos(theta2) * dx + sin(theta2) * dy);

         int ref = int(m.verts.size());
         m.verts.emplace_back(A);
         m.verts.emplace_back(B);
         m.verts.emplace_back(C);

         auto a_norm = Vector3(0, 0, 1);
         m.norms.emplace_back(a_norm);
         m.norms.emplace_back(a_norm);
         m.norms.emplace_back(a_norm);

         m.faces.emplace_back(ref + 0, ref + 1, ref + 2);
      }
   }

   { // Green dot on top of the camera
      auto& m              = meshes[3];
      m.material.diffuse   = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
      m.material.specular  = Vector4f(0.4f, 0.2f, 0.4f, 1.0f);
      m.material.ambient   = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
      m.material.emission  = Vector4f(0.0f, 0.8f, 0.0f, 1.0f);
      m.material.shininess = 0.0f;

      const auto dial_sz     = size / 2.0;
      const auto radius      = 0.5 * dial_sz;
      const auto slice_theta = 2.0 * M_PI / real(cylindar_sides);
      const auto C           = Vector3(0, size + 1e-3, 0);
      const auto dx          = Vector3(1, 0, 0);
      const auto dz          = Vector3(0, 0, 1);

      for(auto i = 0; i < cylindar_sides; ++i) {
         const auto theta1 = (i + 0) * slice_theta;
         const auto theta2 = (i + 1) * slice_theta;
         const auto A      = C + radius * (cos(theta1) * dx + sin(theta1) * dz);
         const auto B      = C + radius * (cos(theta2) * dx + sin(theta2) * dz);

         int ref = int(m.verts.size());
         m.verts.emplace_back(A);
         m.verts.emplace_back(B);
         m.verts.emplace_back(C);

         auto a_norm = Vector3(0, 1, 0);
         m.norms.emplace_back(a_norm);
         m.norms.emplace_back(a_norm);
         m.norms.emplace_back(a_norm);

         m.faces.emplace_back(ref + 1, ref + 0, ref + 2);
      }
   }

   { // Red dot on the side of the camera
      auto& m              = meshes[4];
      m.material.diffuse   = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
      m.material.specular  = Vector4f(0.4f, 0.2f, 0.4f, 1.0f);
      m.material.ambient   = Vector4f(0.2f, 0.2f, 0.2f, 1.0f);
      m.material.emission  = Vector4f(0.8f, 0.0f, 0.0f, 1.0f);
      m.material.shininess = 0.0f;

      const auto dial_sz     = size / 2.0;
      const auto radius      = 0.5 * dial_sz;
      const auto slice_theta = 2.0 * M_PI / real(cylindar_sides);
      const auto C           = Vector3(size + 1e-3, 0, 0);
      const auto dy          = Vector3(0, 1, 0);
      const auto dz          = Vector3(0, 0, 1);

      for(auto i = 0; i < cylindar_sides; ++i) {
         const auto theta1 = (i + 0) * slice_theta;
         const auto theta2 = (i + 1) * slice_theta;
         const auto A      = C + radius * (cos(theta1) * dz + sin(theta1) * dy);
         const auto B      = C + radius * (cos(theta2) * dz + sin(theta2) * dy);

         int ref = int(m.verts.size());
         m.verts.emplace_back(A);
         m.verts.emplace_back(B);
         m.verts.emplace_back(C);

         auto a_norm = Vector3(1, 0, 0);
         m.norms.emplace_back(a_norm);
         m.norms.emplace_back(a_norm);
         m.norms.emplace_back(a_norm);

         m.faces.emplace_back(ref + 1, ref + 0, ref + 2);
      }
   }

   Sprite cam;
   cam.set_meshes(meshes);
   if(cam.fragments().size() == 0)
      throw std::runtime_error("sprite model was empty");
   return cam;
}

// ------------------------------------------------------ Make Model Displaylist

unsigned make_model_displaylist(const Sprite& model)
{
   auto index = 0u;
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   check_and_warn_on_gl_error("before generating display-list");
   index = glGenLists(1);
   check_and_warn_on_gl_error("after generating display-list");
   if(index > 0) {
      glNewList(index, GL_COMPILE);
      render_gl(model);
      glEndList();
      check_and_warn_on_gl_error("after compiling display-list");
   }
#endif
   return index;
}

// --------------------------------------------------- Destroy Model Displaylist

void destroy_model_displaylist(unsigned id)
{
#ifndef USING_OPENGL
   WARN("opengl not compiled in");
#else
   if(id > 0) glDeleteLists(id, 1);
#endif
}

} // namespace perceive
