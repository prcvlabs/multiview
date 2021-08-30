
#pragma once

#include <streambuf>

namespace perceive
{
template<class CharT, class Traits = std::char_traits<CharT>>
class basic_membuf : public std::basic_streambuf<CharT, Traits>
{
 public:
   static_assert(std::is_same_v<CharT, typename Traits::char_type>);
   using char_type = CharT;
   using pos_type  = typename Traits::pos_type;
   using off_type  = typename Traits::off_type;

   // Construction
   basic_membuf() {}

   basic_membuf(char_type* base, std::size_t size)
   {
      this->pubsetbuf(base, std::streamsize(size));
   }

   virtual ~basic_membuf()           = default;
   basic_membuf(const basic_membuf&) = delete;
   basic_membuf& operator=(const basic_membuf&) = delete;

   pos_type capacity() { return this->epptr() - this->pbase(); }

 protected:
   basic_membuf* setbuf(char_type* s, std::streamsize n) override
   {
      if(n < 0) n = 0;
      this->setp(s, s + n);
      this->setg(s, s, s + n);
      return this;
   }

   pos_type seekoff(off_type off,
                    std::ios_base::seekdir dir,
                    std::ios_base::openmode which) override
   {
      auto do_seek = [&](char_type* base,
                         char_type* p,
                         char_type* end,
                         std::ios_base::openmode which) {
         pos_type abs_pos = pos_type(off_type(-1));
         char_type* ptr   = (dir == std::ios_base::beg)
                              ? base
                              : (dir == std::ios_base::cur) ? p : end;
         char_type* new_ptr = ptr + off;
         if(new_ptr >= base and new_ptr <= end) {
            abs_pos = seekpos(new_ptr - base, which);
         }
         return abs_pos;
      };

      pos_type abs_pos = pos_type(off_type(-1));
      if(which & std::ios_base::out) {
         abs_pos = do_seek(
             this->pbase(), this->pptr(), this->epptr(), std::ios_base::out);
      }

      if(which & std::ios_base::in) {
         abs_pos = do_seek(
             this->eback(), this->gptr(), this->egptr(), std::ios_base::in);
      }
      return abs_pos;
   }

   pos_type seekpos(pos_type off, std::ios_base::openmode which) override
   {
      pos_type pos = (off >= capacity()) ? capacity() : off;
      if(pos < 0) pos = 0;
      if(which & std::ios_base::out) {
         this->setp(this->pbase(), this->epptr());
         assert(pos <= std::numeric_limits<int>::max());
         this->pbump(int(pos));
      }
      if(which & std::ios_base::in)
         this->setg(this->eback(), this->eback() + pos, this->egptr());
      return pos;
   }

   std::streamsize showmanyc() override { return this->egptr() - this->gptr(); }

   std::streamsize xsputn(const char_type* s, std::streamsize count) override
   {
      if(count <= 0) return 0;
      auto avail = this->epptr() - this->pptr();
      if(avail <= 0) return 0;
      if(count > avail) count = avail;
      if(count > std::numeric_limits<int>::max())
         count = std::numeric_limits<int>::max();
      memcpy(this->pptr(), s, size_t(count));
      this->pbump(int(count));
      return count;
   }

   std::streamsize xsgetn(char_type* s, std::streamsize count) override
   {
      if(count <= 0) return 0;
      auto avail = showmanyc();
      if(count > avail) count = avail;
      if(count > std::numeric_limits<int>::max())
         count = std::numeric_limits<int>::max();
      memcpy(s, this->gptr(), size_t(count));
      this->gbump(int(count));
      return count;
   }
};

} // namespace perceive
