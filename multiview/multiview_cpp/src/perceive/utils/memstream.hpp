
#pragma once

#include "detail/membuf.hpp"

namespace perceive
{
template<class CharT,
         class Traits    = std::char_traits<CharT>,
         class Allocator = std::allocator<CharT>>
class basic_omemstream : public std::basic_ostream<CharT, Traits>
{
 public:
   static_assert(std::is_same_v<CharT, typename Traits::char_type>);
   using char_type      = CharT;
   using traits_type    = Traits;
   using int_type       = typename Traits::int_type;
   using pos_type       = typename Traits::pos_type;
   using off_type       = typename Traits::off_type;
   using allocator_type = Allocator;
   using membuf_type    = basic_membuf<CharT, Traits>;

 private:
   membuf_type rdbuf_;

 public:
   basic_omemstream()
       : std::basic_ostream<CharT, Traits>(&rdbuf_)
   {}
   basic_omemstream(char_type* mem, size_t size)
       : std::basic_ostream<CharT, Traits>(&rdbuf_)
       , rdbuf_(mem, size)
   {}
   basic_omemstream(const basic_omemstream&) = delete;
   basic_omemstream(basic_omemstream&& o) { swap(o); }
   virtual ~basic_omemstream() = default;

   basic_omemstream& operator=(const basic_omemstream&) = delete;
   basic_omemstream& operator=(basic_omemstream&& o) { swap(o); }

   pos_type capacity() { return rdbuf_.capacity(); }
};

template<class CharT, class Traits, class Alloc>
void swap(basic_omemstream<CharT, Traits, Alloc>& lhs,
          basic_omemstream<CharT, Traits, Alloc>& rhs)
{
   lhs.swap(rhs);
}

template<class CharT,
         class Traits    = std::char_traits<CharT>,
         class Allocator = std::allocator<CharT>>
class basic_imemstream : public std::basic_istream<CharT, Traits>
{
 public:
   static_assert(std::is_same_v<CharT, typename Traits::char_type>);
   using char_type      = CharT;
   using traits_type    = Traits;
   using int_type       = typename Traits::int_type;
   using pos_type       = typename Traits::pos_type;
   using off_type       = typename Traits::off_type;
   using allocator_type = Allocator;
   using membuf_type    = basic_membuf<CharT, Traits>;

 private:
   membuf_type rdbuf_;

 public:
   basic_imemstream()
       : std::basic_istream<CharT, Traits>(&rdbuf_)
   {}

   basic_imemstream(const char_type* mem, size_t size)
       : std::basic_istream<CharT, Traits>(&rdbuf_)
       , rdbuf_(const_cast<char_type*>(mem), size)
   {}
   basic_imemstream(const basic_imemstream&) = delete;
   basic_imemstream(basic_imemstream&& o) { swap(o); }
   virtual ~basic_imemstream() = default;

   basic_imemstream& operator=(const basic_imemstream&) = delete;
   basic_imemstream& operator=(basic_imemstream&& o) { swap(o); }

   pos_type capacity() { return rdbuf_.capacity(); }
};

template<class CharT, class Traits, class Alloc>
void swap(basic_imemstream<CharT, Traits, Alloc>& lhs,
          basic_imemstream<CharT, Traits, Alloc>& rhs)
{
   lhs.swap(rhs);
}

template<class CharT,
         class Traits    = std::char_traits<CharT>,
         class Allocator = std::allocator<CharT>>
class basic_memstream : public std::basic_iostream<CharT, Traits>
{
 public:
   static_assert(std::is_same_v<CharT, typename Traits::char_type>);
   using char_type      = CharT;
   using traits_type    = Traits;
   using int_type       = typename Traits::int_type;
   using pos_type       = typename Traits::pos_type;
   using off_type       = typename Traits::off_type;
   using allocator_type = Allocator;
   using membuf_type    = basic_membuf<CharT, Traits>;

 private:
   membuf_type rdbuf_;

 public:
   basic_memstream()
       : std::basic_iostream<CharT, Traits>(&rdbuf_)
   {}
   basic_memstream(char_type* mem, size_t size)
       : std::basic_iostream<CharT, Traits>(&rdbuf_)
       , rdbuf_(mem, size)
   {}
   basic_memstream(const basic_memstream&) = delete;
   basic_memstream(basic_memstream&& o) { swap(o); }
   virtual ~basic_memstream() = default;

   basic_memstream& operator=(const basic_memstream&) = delete;
   basic_memstream& operator=(basic_memstream&& o) { swap(o); }

   pos_type capacity() { return rdbuf_.capacity(); }
};

template<class CharT, class Traits, class Alloc>
void swap(basic_memstream<CharT, Traits, Alloc>& lhs,
          basic_memstream<CharT, Traits, Alloc>& rhs)
{
   lhs.swap(rhs);
}

using omemstream = basic_omemstream<char>;
using imemstream = basic_imemstream<char>;
using memstream  = basic_memstream<char>;

} // namespace perceive
