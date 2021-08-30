
#include <cassert>
#include <cmath>
#include <cstdio>
#include <type_traits>

namespace perceive::detail
{
// --------------------------------------------------------------------- Bitcast
//
template<typename To,
         typename From,
         typename = std::enable_if<(sizeof(To) == sizeof(From))
                                   && std::is_trivially_copyable<To>::value
                                   && std::is_trivially_copyable<From>::value>>
To bit_cast_(const From& src) noexcept
{
   To dst;
   std::memcpy(&dst, &src, sizeof(To)); // The compiler optimizes this away
   return dst;
}

// ------------------------------------------------------------------ pack-float

// F ==> floating point type
// I ==> unsigned integer type
// EXP_DIG ==> digits in the exponent.
//             Should be std::numeric_limits<F>::exponent
template<typename F, int EXP_DIG> auto pack_float(const F value) noexcept
{
   using I =
       typename std::conditional<sizeof(F) == 8, uint64_t, uint32_t>::type;
   static_assert(std::is_integral<I>::value);
   static_assert(std::is_floating_point<F>::value);

   constexpr int total_bits      = sizeof(F) * 8;
   constexpr int exponent_bits   = EXP_DIG;
   constexpr int fraction_bits   = total_bits - exponent_bits - 1;
   constexpr I fraction_mask     = (I(1) << fraction_bits) - 1;
   constexpr I exponent_mask     = ~fraction_mask & ~(I(1) << (total_bits - 1));
   constexpr int exponent_offset = ((1 << (exponent_bits - 1)) - 1);

   constexpr int exponent_max = (1 << (EXP_DIG - 1));
   constexpr int exponent_min = 2 - exponent_max;

   bool sign_bit = false;
   F fraction    = F(0.0);
   int exponent  = 0;
   I packed      = 0;

   if(!std::isfinite(value)) {
      if(std::isnan(value)) {
         packed = exponent_mask | (I(1) << (fraction_bits - 1));
      } else {
         sign_bit = std::signbit(value);
         packed   = exponent_mask;
         if(sign_bit) packed |= (I(1) << (total_bits - 1));
      }

   } else {
      // Unpack the value
      exponent = 0;
      fraction = std::frexp(value, &exponent);
      if(fraction != F(0.0)) exponent -= 1;
      sign_bit = std::signbit(fraction);

      const bool is_denorm = exponent < exponent_min;
      if(is_denorm) { // Handle denormalized numbers
         while(exponent < exponent_min) {
            fraction /= F(2.0);
            exponent += 1;
         }
      }

      const bool is_zero = (fraction == F(0.0));

      if(is_zero) {
         packed = 0; // all good
      } else {
         constexpr int shift = fraction_bits;
         constexpr F mult    = F(I(1) << shift);
         auto y              = F(2.0) * (sign_bit ? -fraction : fraction);
         packed              = I(mult * (y - F(I(y))));
         assert(packed >= 0 and packed < (I(1) << shift));

         // Remove any excess precision
         packed &= fraction_mask;

         // Add the exponent
         I out_exp = 0;
         if(!is_denorm) {
            if(exponent >= exponent_max) exponent = exponent_max - 1;
            if(exponent < -exponent_max + 2) exponent = -exponent_max + 2;
            out_exp = (I(exponent + exponent_offset) << fraction_bits);
         }

         assert(out_exp == (out_exp & exponent_mask));
         assert((out_exp ^ exponent_mask) != 0); // that would be infinity

         packed |= out_exp;
      }

      // Add sign bit
      if(sign_bit) packed |= (I(1) << (total_bits - 1));
   }

   return packed;
}

// ---------------------------------------------------------------- unpack-float

template<typename I, int EXP_DIG> auto unpack_float(const I packed) noexcept
{
   using F = typename std::conditional<sizeof(I) == 8, double, float>::type;
   static_assert(std::is_integral<I>::value);
   static_assert(std::is_floating_point<F>::value);

   constexpr int total_bits      = sizeof(I) * 8;
   constexpr int exponent_bits   = EXP_DIG;
   constexpr int fraction_bits   = total_bits - exponent_bits - 1;
   constexpr I fraction_mask     = (I(1) << fraction_bits) - 1;
   constexpr I sign_mask         = I(1) << (total_bits - 1);
   constexpr I exponent_mask     = ~fraction_mask & ~sign_mask;
   constexpr int exponent_offset = ((1 << (exponent_bits - 1)) - 1);

   const bool sign_bit = (packed & sign_mask) != 0;

   // infinity and NAN
   const bool is_finite = ((packed & exponent_mask) ^ exponent_mask) != 0;
   if(!is_finite) {
      if((packed & fraction_mask) != 0)
         return std::numeric_limits<F>::quiet_NaN();
      else if(!sign_bit)
         return std::numeric_limits<F>::infinity();
      else
         return -std::numeric_limits<F>::infinity();
   }

   // Onto finite values
   int exponent0 = ((packed & exponent_mask) >> fraction_bits);
   int exponent  = exponent0 - exponent_offset;

   const bool is_denorm = (exponent0 == 0);

   // Handle 0.0 and -0.0
   if(exponent0 == 0 and (packed & fraction_mask) == 0)
      return sign_bit ? -F(0.0) : F(0.0);

   // Handle 1.0 and -1.0
   if(exponent == 0 and (packed & fraction_mask) == 0 and !is_denorm)
      return sign_bit ? -F(1.0) : F(1.0);

   const I packed_fraction0 = (packed & fraction_mask) << (exponent_bits + 1);

   constexpr int shift    = fraction_bits;
   constexpr I shift_mask = ((I(1) << shift) - 1) << (total_bits - shift);
   constexpr F mult       = F(I(1) << shift);
   constexpr F div        = F(1.0) / mult;

   auto frac_int = I((packed_fraction0 & shift_mask) >> (total_bits - shift));

   F fraction = F(0.5) * (F(frac_int) * div + (is_denorm ? F(0.0) : F(1.0)));

   F out = std::ldexp(fraction, exponent + 1);

   return sign_bit ? -out : out;
}

// @see
// https://stackoverflow.com/questions/1659440/32-bit-to-16-bit-floating-point-conversion
// class Float16Compressor
// {
//    union Bits
//    {
//       float f;
//       int32_t si;
//       uint32_t ui;
//    };

//    static int const shift     = 13;
//    static int const shiftSign = 16;

//    static int32_t const infN  = 0x7F800000; // flt32 infinity
//    static int32_t const maxN  = 0x477FE000; // max flt16 normal as a flt32
//    static int32_t const minN  = 0x38800000; // min flt16 normal as a flt32
//    static int32_t const signN = 0x80000000; // flt32 sign bit

//    static int32_t const infC = infN >> shift;
//    static int32_t const nanN = (infC + 1)
//                                << shift; // minimum flt16 nan as a flt32
//    static int32_t const maxC  = maxN >> shift;
//    static int32_t const minC  = minN >> shift;
//    static int32_t const signC = signN >> shiftSign; // flt16 sign bit

//    static int32_t const mulN = 0x52000000; // (1 << 23) / minN
//    static int32_t const mulC = 0x33800000; // minN / (1 << (23 - shift))

//    static int32_t const subC = 0x003FF; // max flt32 subnormal down shifted
//    static int32_t const norC = 0x00400; // min flt32 normal down shifted

//    static int32_t const maxD = infC - maxC - 1;
//    static int32_t const minD = minC - subC - 1;

//  public:
//    static uint16_t compress(float value)
//    {
//       Bits v, s;
//       v.f           = value;
//       uint32_t sign = v.si & signN;
//       v.si ^= sign;
//       sign >>= shiftSign; // logical shift
//       s.si = mulN;
//       s.si = s.f * v.f; // correct subnormals
//       v.si ^= (s.si ^ v.si) & -(minN > v.si);
//       v.si ^= (infN ^ v.si) & -((infN > v.si) & (v.si > maxN));
//       v.si ^= (nanN ^ v.si) & -((nanN > v.si) & (v.si > infN));
//       v.ui >>= shift; // logical shift
//       v.si ^= ((v.si - maxD) ^ v.si) & -(v.si > maxC);
//       v.si ^= ((v.si - minD) ^ v.si) & -(v.si > subC);
//       return v.ui | sign;
//    }

//    static float decompress(uint16_t value)
//    {
//       Bits v;
//       v.ui         = value;
//       int32_t sign = v.si & signC;
//       v.si ^= sign;
//       sign <<= shiftSign;
//       v.si ^= ((v.si + minD) ^ v.si) & -(v.si > subC);
//       v.si ^= ((v.si + maxD) ^ v.si) & -(v.si > maxC);
//       Bits s;
//       s.si = mulC;
//       s.f *= v.si;
//       int32_t mask = -(norC > v.si);
//       v.si <<= shift;
//       v.si ^= (s.si ^ v.si) & mask;
//       v.si |= sign;
//       return v.f;
//    }
// };

} // namespace perceive::detail

namespace perceive
{
// inline uint16_t compress_f16(float value) noexcept
// {
// #if defined __STDC_IEC_559__ && !defined USE_CPU_FLOAT_PACKING
//    return detail::Float16Compressor::compress(value);
// #else
//    FATAL(format("not implemented"));
//    return detail::Float16Compressor::compress(value);
// #endif
// }

// inline float decompress_f16(uint16_t value) noexcept
// {
// #if defined __STDC_IEC_559__ && !defined USE_CPU_FLOAT_PACKING
//    return detail::Float16Compressor::decompress(value);
// #else
//    FATAL(format("not implemented"));
//    return detail::Float16Compressor::decompress(value);
// #endif
// }

template<typename F> auto pack_float(F x) noexcept
{
#if defined __STDC_IEC_559__ && !defined USE_CPU_FLOAT_PACKING
   using I = typename std::conditional_t<sizeof(F) == 8, uint64_t, uint32_t>;
   return detail::bit_cast_<I, F>(x);
#else
   if constexpr(sizeof(F) == 8) return detail::pack_float<F, 11>(x);
   return detail::pack_float<F, 8>(x);
#endif
}

template<typename I> auto unpack_float(I x) noexcept
{
   using F = typename std::conditional_t<sizeof(I) == 8, double, float>;
#if defined __STDC_IEC_559__ && !defined USE_CPU_FLOAT_PACKING
   return detail::bit_cast_<F, I>(x);
#else
   if constexpr(sizeof(F) == 8) return detail::unpack_float<I, 11>(x);
   return detail::unpack_float<I, 8>(x);
#endif
}

inline uint32_t pack_f32(float x) noexcept { return pack_float(x); }
inline uint64_t pack_f64(double x) noexcept { return pack_float(x); }
inline float unpack_f32(uint32_t x) noexcept { return unpack_float(x); }
inline double unpack_f64(uint64_t x) noexcept { return unpack_float(x); }

constexpr bool is_cpu_ieee754_packing() noexcept
{
#if defined __STDC_IEC_559__ && !defined USE_CPU_FLOAT_PACKING
   return false;
#else
   return true;
#endif
}

} // namespace perceive
