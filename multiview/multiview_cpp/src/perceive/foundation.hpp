
#pragma once

#include "config.hpp"

// ------------------------------------------------------------- Likely/unlikely

#if defined(__clang__) || defined(__GNUC__)
#define branch_is_likely(x) __builtin_expect(!!(x), 1)
#define branch_is_unlikely(x) __builtin_expect(!!(x), 0)
#else
#define branch_is_likely(x) (!!(x))
#define branch_is_unlikely(x) (!!(x))
#endif

// -------------------------------------------------------------- C/C++ Includes

#define _FILE_OFFSET_BITS 64

#ifndef __cplusplus
#error "this is a c++ only include"
#endif

// C includes
#include <assert.h>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
// C++ includes
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <unordered_map>
#include <unordered_set>

// ------------------------------------------------------------------ Beyond std

#include "fmt/format.h"
#include "range/v3/all.hpp"

// ------------------------------------------------------------ Perceive headers

#include "utils/stdcpp-compat.hpp"

#include "utils/concepts.hpp"
#include "utils/container.hpp"
#include "utils/copying-ptr.hpp"
#include "utils/exit-helpers.hpp"
#include "utils/index.hpp"
#include "utils/logger.hpp"
#include "utils/memory.hpp"
#include "utils/spin-lock.hpp"
#include "utils/system.hpp"
#include "utils/threads.hpp"
#include "utils/type-trait-helpers.hpp"

namespace views   = ranges::views;
namespace actions = ranges::actions;

namespace perceive
{
using fmt::format;

using std::array;
using std::string;
using std::string_view;
using std::vector;

using std::cout;
using std::endl;

using std::cbegin;
using std::cend;

using std::round;

using namespace std::string_literals;

using real = double;

template<typename T>
using deleted_unique_ptr = std::unique_ptr<T, std::function<void(T*)>>;

// common_type means here: the type of the mixed type arithmetic express:
//    ptrdiff_t(0) + std::make_size_t<std::size_t>(0)
using ssize_t
    = std::common_type_t<std::ptrdiff_t, std::make_signed_t<std::size_t>>;

// -------------------------------------------------------------------- Aliasing

#define restrict __restrict

// -- Adjust GSL Expects and Ensures macros
#ifdef Expects
#undef Expects
#endif
#define Expects(cond)                  \
   {                                   \
      if(!branch_is_likely(cond)) {    \
         assert(false);                \
         FATAL("Precondition failed"); \
      }                                \
   }

#ifdef Ensures
#undef Ensures
#endif
#define Ensures(cond)                   \
   {                                    \
      if(!branch_is_likely(cond)) {     \
         assert(false);                 \
         FATAL("Postcondition failed"); \
      }                                 \
   }

// A raw pointer that is OWNED by something (the struct/class)
template<class T> using owned_ptr = T*;

// A pointer that is not owned.
template<class T> using raw_ptr = T*;

// A unique_ptr that is copied like a standard member
template<typename T> using copying_ptr = nonstd::copying_ptr<T>;

/**
 * Ownership...
 * + unique_ptr, unique ownership
 * + shared_ptr, shared ownership
 * + owner, object/function owns the pointer
 * + pointer, non-owned pointer that could dangle
 * + copying_ptr, copyable unique ownership -- a member on the heap
 * ...
 * + not_null<.> pointer that's never null
 */

using std::make_shared;
using std::make_unique;
using std::shared_ptr;
using std::unique_ptr;

template<class Key, class T> using hashmap = std::unordered_map<Key, T>;
template<class Key> using hashset          = std::unordered_set<Key>;

} // namespace perceive
