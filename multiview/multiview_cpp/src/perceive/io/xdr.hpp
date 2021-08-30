
#include "perceive/foundation.hpp"
#include "perceive/geometry.hpp"

#include <rpc/xdr.h>

namespace perceive
{
struct XdrWrapper
{
   XDR xdrs;
   enum xdr_op op;

   // @see http://people.redhat.com/rjones/xdr_tests/
   // @see http://man7.org/linux/man-pages/man3/xdr.3.html
   // XDR_ENCODE, XDR_DECODE
   XdrWrapper(FILE* fp, enum xdr_op op_)
       : op(op_)
   {
      xdrstdio_create(&xdrs, fp, op);
   }
   ~XdrWrapper() { xdr_destroy(&xdrs); }

   bool is_encode() const noexcept { return op == XDR_ENCODE; }
   bool is_decode() const noexcept { return op == XDR_DECODE; }
};

#define XDR_DEFINE(type, f)                                       \
   inline void xdr_encode(XdrWrapper& x, const type& val)         \
   {                                                              \
      type v = val;                                               \
      if(!f(&x.xdrs, &v))                                         \
         throw std::runtime_error("xdr failed to encode " #type); \
   }                                                              \
                                                                  \
   inline void xdr_decode(XdrWrapper& x, type& val)               \
   {                                                              \
      if(!f(&x.xdrs, &val))                                       \
         throw std::runtime_error("xdr failed to decode " #type); \
   }

XDR_DEFINE(char, xdr_char)
XDR_DEFINE(short, xdr_short)
XDR_DEFINE(int, xdr_int)
XDR_DEFINE(long, xdr_long)
XDR_DEFINE(unsigned char, xdr_u_char)
XDR_DEFINE(unsigned short, xdr_u_short)
XDR_DEFINE(unsigned int, xdr_u_int)
XDR_DEFINE(unsigned long, xdr_u_long)
XDR_DEFINE(float, xdr_float)
XDR_DEFINE(double, xdr_double)

#undef XDR_DEFINE

// ---------------------------------------------------------- Encode/Decode bool

inline void xdr_encode(XdrWrapper& x, const bool& val)
{
   bool_t v = val;
   if(!xdr_bool(&x.xdrs, &v))
      throw std::runtime_error("xdr failed to encode "
                               "bool");
}

inline void xdr_decode(XdrWrapper& x, bool& val)
{
   bool_t v = 0;
   if(!xdr_bool(&x.xdrs, &v))
      throw std::runtime_error("xdr failed to decode "
                               "bool");
   val = v;
}

// -------------------------------------------------------- Encode/Decode string

inline void xdr_encode(XdrWrapper& x, const string& val)
{
   unsigned int sz = unsigned(val.size() + 1);
   if(size_t(sz) != val.size() + 1) throw std::runtime_error("string too big!");
   char* v = static_cast<char*>(malloc(sizeof(char) * (sz)));
   if(v == nullptr) throw std::runtime_error("out of memory");
   strcpy(v, val.c_str());

   try {
      xdr_encode(x, sz);
      if(!xdr_string(&x.xdrs, &v, sz))
         throw std::runtime_error("xdr failed to encode string");
      free(v);
   } catch(std::runtime_error& e) {
      free(v);
      throw e;
   } catch(std::exception& e) {
      free(v);
      throw e;
   } catch(...) {
      free(v);
      throw std::runtime_error("unknown error");
   }
}

inline void xdr_decode(XdrWrapper& x, string& val)
{
   unsigned int len{0};
   xdr_decode(x, len);
   char* v = static_cast<char*>(malloc(sizeof(char) * (len)));
   if(v == nullptr) throw std::runtime_error("out of memory");

   try {
      if(!xdr_string(&x.xdrs, &v, len))
         throw std::runtime_error("xdr failed to decode string");
      val = v;
      free(v);
   } catch(std::runtime_error& e) {
      free(v);
      throw e;
   } catch(std::exception& e) {
      free(v);
      throw e;
   } catch(...) {
      free(v);
      throw std::runtime_error("unknown error");
   }
}

// ---------------------------------------------------------- Encode/Decode AABB

template<typename T> inline void xdr_encode(XdrWrapper& x, const AABBT<T>& val)
{
   AABBT<T> v = val;
   xdr_encode(x, v.left);
   xdr_encode(x, v.top);
   xdr_encode(x, v.right);
   xdr_encode(x, v.bottom);
}

template<typename T> inline void xdr_decode(XdrWrapper& x, AABBT<T>& val)
{
   xdr_decode(x, val.left);
   xdr_decode(x, val.top);
   xdr_decode(x, val.right);
   xdr_decode(x, val.bottom);
}

// ------------------------------------------------------- Encode/Decode Vector2

template<typename T>
inline void xdr_encode(XdrWrapper& x, const Vector2T<T>& val)
{
   xdr_encode(x, val.x);
   xdr_encode(x, val.y);
}

template<typename T> inline void xdr_decode(XdrWrapper& x, Vector2T<T>& val)
{
   xdr_decode(x, val.x);
   xdr_decode(x, val.y);
}

// ------------------------------------------------------- Encode/Decode Vector3

template<typename T>
inline void xdr_encode(XdrWrapper& x, const Vector3T<T>& val)
{
   xdr_encode(x, val.x);
   xdr_encode(x, val.y);
   xdr_encode(x, val.z);
}

template<typename T> inline void xdr_decode(XdrWrapper& x, Vector3T<T>& val)
{
   xdr_decode(x, val.x);
   xdr_decode(x, val.y);
   xdr_decode(x, val.z);
}

// ------------------------------------------------------- Encode/Decode Vector4

template<typename T>
inline void xdr_encode(XdrWrapper& x, const Vector4T<T>& val)
{
   xdr_encode(x, val.x);
   xdr_encode(x, val.y);
   xdr_encode(x, val.z);
   xdr_encode(x, val.w);
}

template<typename T> inline void xdr_decode(XdrWrapper& x, Vector4T<T>& val)
{
   xdr_decode(x, val.x);
   xdr_decode(x, val.y);
   xdr_decode(x, val.z);
   xdr_decode(x, val.w);
}

// ---------------------------------------------------- Encode/Decode Quaternion

template<typename T>
inline void xdr_encode(XdrWrapper& x, const QuaternionT<T>& val)
{
   xdr_encode(x, val.x);
   xdr_encode(x, val.y);
   xdr_encode(x, val.z);
   xdr_encode(x, val.w);
}

template<typename T> inline void xdr_decode(XdrWrapper& x, QuaternionT<T>& val)
{
   xdr_decode(x, val.x);
   xdr_decode(x, val.y);
   xdr_decode(x, val.z);
   xdr_decode(x, val.w);
}

} // namespace perceive
