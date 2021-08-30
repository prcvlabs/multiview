
// NINJA_BUILD_RELEASE_O3

#include "stdinc.hpp"

namespace perceive
{
// ----------------------------------------------------------- Back Project Quad

/*
Clear (ra);
Clear (rb);
Clear (rc);
Clear (rd);

ma = {max, may, maz};
mb = {mbx, mby, mbz};
mc = {mcx, mcy, mcz};
md = {mdx, mdy, mdz};

e = rb*mb - ra*ma;
f = rc*mc - rb*mb;
g = rd*md - rc*mc;
h = ra*ma - rd*md;

ra > 0;
rb > 0;
rc > 0;
rd > 0;

Print["Cross-products are all the same"]

ecf = rb*rc*Cross[mb, mc] - rb*rb*Cross[mb, mb] -
   ra*rc*Cross[ma, mc] + ra*rb*Cross[ma, mb];
fcg = rc*rd*Cross[mc, md] - rc*rc*Cross[mc, mc] -
   rb*rd*Cross[mb, md] + rb*rc*Cross[mb, mc];
gch = rd*ra*Cross[md, ma] - rd*rd*Cross[md, md] -
   rc*ra*Cross[mc, ma] + rc*rd*Cross[mc, md];
hce = ra*rb*Cross[ma, mb] - ra*ra*Cross[ma, ma] -
   rd*rb*Cross[md, mb] + rd*ra*Cross[md, ma];

z1 = ecf == fcg;
z2 = fcg == gch;
z3 = gch == hce;
z4 = hce == ecf;

Solve[{z1, z2, z3, z4}, {rb, rc, rd}]

Print["Dot-products are 0 -- maybe these formula will be useful"];
eqef = rb*rc mb.mc - rb*rb mb.mb - ra*rc ma.mc + ra*rb ma.mb == 0;
eqfg = rc*rd mc.md - rc*rc mc.mc - rb*rd mb.md + rb*rc mb.mc == 0;
eqgh = rd*ra md.ma - rd*rd md.md - rc*ra mc.ma + rc*rd mc.md == 0;
eqhe = ra*rb ma.mb - ra*ra ma.ma - rd*rb md.mb + rd*ra md.ma == 0;

(*
Solve[{eqef},{rc}]
Solve[{eqfg},{rd}]
*)
*/

array<Vector3, 4> back_project_kite(const array<Vector3, 4>& quad_2d,
                                    double distance_to_centre)
{
   // Camera centre
   const auto& C = Vector3(0.0, 0.0, 0.0);

   const auto& ray_a = quad_2d[0];
   const auto& ray_b = quad_2d[1];
   const auto& ray_c = quad_2d[2];
   const auto& ray_d = quad_2d[3];

   // Optimzing the above equations... by precalculating terms
   const Vector3& ma = ray_a;
   const Vector3& mb = ray_b;
   const Vector3& mc = ray_c;
   const Vector3& md = ray_d;

   const auto maz_mcy_mdx = ma.z * mc.y * md.x;
   const auto may_mcz_mdx = ma.y * mc.z * md.x;
   const auto maz_mcx_mdy = ma.z * mc.x * md.y;
   const auto max_mcz_mdy = ma.x * mc.z * md.y;
   const auto may_mcx_mdz = ma.y * mc.x * md.z;
   const auto max_mcy_mdz = ma.x * mc.y * md.z;

   const auto mbz_mcy_mdx = mb.z * mc.y * md.x;
   const auto mby_mcz_mdx = mb.y * mc.z * md.x;
   const auto mbz_mcx_mdy = mb.z * mc.x * md.y;
   const auto mbx_mcz_mdy = mb.x * mc.z * md.y;
   const auto mby_mcx_mdz = mb.y * mc.x * md.z;
   const auto mbx_mcy_mdz = mb.x * mc.y * md.z;

   const auto rb_numer = maz_mcy_mdx - may_mcz_mdx - maz_mcx_mdy + max_mcz_mdy
                         + may_mcx_mdz - max_mcy_mdz;
   const auto rb_denom_inv = 1.0
                             / (mbz_mcy_mdx - mby_mcz_mdx - mbz_mcx_mdy
                                + mbx_mcz_mdy + mby_mcx_mdz - mbx_mcy_mdz);

   const auto rb_k = rb_numer * rb_denom_inv;

   const auto rc_k = -rb_denom_inv
                     * (-(ma.z * mb.y * md.x) + (ma.y * mb.z * md.x)
                        + (ma.z * mb.x * md.y) - (ma.x * mb.z * md.y)
                        - (ma.y * mb.x * md.z) + (ma.x * mb.y * md.z));
   const auto rd_k = -rb_denom_inv
                     * (-(ma.z * mb.y * mc.x) + (ma.y * mb.z * mc.x)
                        + (ma.z * mb.x * mc.y) - (ma.x * mb.z * mc.y)
                        - (ma.y * mb.x * mc.z) + (ma.x * mb.y * mc.z));

   // Actually,
   // ra_k = 1. (Assumed)
   // rb_k = dot(ma, cross(md, mc)) / dot(mb, cross(md, mc))
   // rc_k = dot(md, cross(ma, mb)) / dot(mb, cross(md, mc))
   // rd_k = dot(mc, cross(ma, mb)) / dot(mb, cross(md, mc))

   auto calc_ra = [&](double ro, const Vector3& C) {
      auto rc    = ro * rc_k;
      Vector3 a1 = ro * ma + C;
      Vector3 c1 = rc * mc + C;

      // Now adjust the result so that its centre is at the input distance
      Vector3 wrong_centre = (a1 + 0.5 * (c1 - a1));
      Vector3 ray_o{(wrong_centre - C).normalised()};

      // This is now the correct centre -- 'o' for the 'origin of the quad'
      Vector3 o = ro * ray_o + C;

      // This is the line (a-c)
      const Vector3& mn = a1 - c1;

      // Now, we want to travel from o, down ac, until we reach a point on
      // ra*ma +p
      return (o.x - C.x + mn.x * (C.y - o.y) / mn.y)
             / (ma.x - mn.x * ma.y / mn.y);
   };

   // auto ra = calc_ra(distance_to_centre, focal_point, linea.n, linec.n);
   auto ra = -calc_ra(distance_to_centre, C);
   auto rb = ra * rb_k;
   auto rc = ra * rc_k;
   auto rd = ra * rd_k;

   array<Vector3, 4> res{{ra * ray_a, rb * ray_b, rc * ray_c, rd * ray_d}};

   if(false) {
      // for debugging
      printf(
          "\n// -- Unproject kite debug: %s, %d -- //\n", __FILE__, __LINE__);

      printf("Input quad: \n");
      printf("   a = ");
      quad_2d[0].print();
      printf("   b = ");
      quad_2d[1].print();
      printf("   c = ");
      quad_2d[2].print();
      printf("   d = ");
      quad_2d[3].print();

      printf("Values: \n");
      printf("   rb_k =%f\n", rb_k);
      printf("   rc_k =%f\n", rc_k);
      printf("   rd_k =%f\n", rd_k);

      printf("3d quad: \n");
      printf("   A = ");
      res[0].print();
      printf("   B = ");
      res[1].print();
      printf("   C = ");
      res[2].print();
      printf("   D = ");
      res[3].print();

      printf("dot-products: \n");
      printf("   (A-B).(B-C) =%f\n", (res[0] - res[1]).dot(res[1] - res[2]));
      printf("   (B-C).(C-D) =%f\n", (res[1] - res[2]).dot(res[2] - res[3]));
      printf("   (C-D).(D-A) =%f\n", (res[2] - res[3]).dot(res[3] - res[0]));
      printf("   (D-A).(A-B) =%f\n", (res[3] - res[0]).dot(res[0] - res[1]));

      auto theta = [&](Vector3 a, Vector3 b) {
         auto cos_t = dot(a.normalized(), b.normalized());
         return to_degrees(acos(clamp(cos_t, -1.0, 1.0)));
      };
      printf("angles: \n");
      printf("   (A-B).(B-C) =%f\n", theta(res[0] - res[1], res[1] - res[2]));
      printf("   (B-C).(C-D) =%f\n", theta(res[1] - res[2], res[2] - res[3]));
      printf("   (C-D).(D-A) =%f\n", theta(res[2] - res[3], res[3] - res[0]));
      printf("   (D-A).(A-B) =%f\n", theta(res[3] - res[0], res[0] - res[1]));

      printf("cross-products: \n");
      printf("   (A-B)x(B-C) = ");
      (res[0] - res[1]).cross(res[1] - res[2]).normalised().print();
      printf("   (B-C)x(C-D) = ");
      (res[1] - res[2]).cross(res[2] - res[3]).normalised().print();
      printf("   (C-D)x(D-A) = ");
      (res[2] - res[3]).cross(res[3] - res[0]).normalised().print();
      printf("   (D-A)x(A-B) = ");
      (res[3] - res[0]).cross(res[0] - res[1]).normalised().print();

      printf("size of each size: \n");
      printf("   norm AB =%f\n", (res[0] - res[1]).norm());
      printf("   norm BC =%f\n", (res[1] - res[2]).norm());
      printf("   norm CD =%f\n", (res[2] - res[3]).norm());
      printf("   norm DA =%f\n", (res[3] - res[0]).norm());

      printf("centre-point: \n");
      Vector3 centre_ac = res[0] + 0.5 * (res[2] - res[0]);
      Vector3 centre_bd = res[1] + 0.5 * (res[3] - res[1]);
      printf("   centre AC (%f) ", (centre_ac - C).norm());
      centre_ac.print();
      printf("   centre BD (%f) ", (centre_bd - C).norm());
      centre_bd.print();

      printf("Camera centre: \n");
      printf("   used: ");
      C.print();

      printf("// -- ~ - //\n");
   }

   return res;
}

} // namespace perceive
