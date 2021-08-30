
#include "base64.hpp"

using std::string;
using std::string_view;
using std::vector;

/**
 * This code is adapted from Apache's  ap_base64.c. Please see the Apache
 * license below.
 */

/* ====================================================================
 * Copyright (c) 1995-1999 The Apache Group.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * 3. All advertising materials mentioning features or use of this
 *    software must display the following acknowledgment:
 *    "This product includes software developed by the Apache Group
 *    for use in the Apache HTTP server project (http://www.apache.org/)."
 *
 * 4. The names "Apache Server" and "Apache Group" must not be used to
 *    endorse or promote products derived from this software without
 *    prior written permission. For written permission, please contact
 *    apache@apache.org.
 *
 * 5. Products derived from this software may not be called "Apache"
 *    nor may "Apache" appear in their names without prior written
 *    permission of the Apache Group.
 *
 * 6. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 *    "This product includes software developed by the Apache Group
 *    for use in the Apache HTTP server project (http://www.apache.org/)."
 *
 * THIS SOFTWARE IS PROVIDED BY THE APACHE GROUP ``AS IS'' AND ANY
 * EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE APACHE GROUP OR
 * ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 * ====================================================================
 *
 * This software consists of voluntary contributions made by many
 * individuals on behalf of the Apache Group and was originally based
 * on public domain software written at the National Center for
 * Supercomputing Applications, University of Illinois, Urbana-Champaign.
 * For more information on the Apache Group and the Apache HTTP server
 * project, please see <http://www.apache.org/>.
 *
 */

#include "base64.hpp"
//#include "variant.hpp"

/* aaaack but it's fast and const should make it shared text page. */
static const unsigned char pr2six[256] = {
    /* ASCII table */
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, // NULL-SI
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, // DLE-US
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 62, 64, 64, 64, 63, // SPACE-'/'
    52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 64, 64, 64, 64, 64, 64, // '0'-'?'
    64, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, // '@'-'O'
    15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 64, 64, 64, 64, 64, // 'P'-'_'
    64, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, // '`'-'o'
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 64, 64, 64, 64, 64, // 'p'-'DEL'
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
    64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64};

static const char* codebook
    = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// ------------------------------------------------------------------- is_base64

bool is_base64(const string_view sv) noexcept
{
   const uint8_t* const s = reinterpret_cast<const uint8_t*>(sv.data());
   if(*s == '\0') return (sv.size() == 0); // zero length blob

   const uint8_t* p = s;
   uint8_t ch;
   int counter = 0;
   while(true) {
      ch = *p++;
      if(pr2six[ch] <= 63) {
         if(++counter == 4) counter = 0; // we read a group of four
         continue;
      }
      if(ch == '\0') return (counter == 0) and (size_t(p - s) == sv.size() + 1);
      if(ch == '=') break;
      return false;
   }

   // Counter /must/ be either '2' or '3'
   if(counter == 0 || counter == 1) return false;

   // How many '=' signs do we have
   int n_equal_signs = 1;
   while(true) {
      ch = *p++;
      if(ch == '=')
         ++n_equal_signs;
      else if(ch == '\0')
         break;
      else
         return false;
   }

   // The size is wrong
   if(size_t(p - s) != sv.size() + 1) return false;

   // Total length of 's' must be divisible by 4
   // This means 'n_equal_signs' is 1 or 2
   if(n_equal_signs + counter == 4) return true;

   return false;
}

// -------------------------------------------------------- base64_decode_length

size_t base64_decode_length(const string_view sv) noexcept(false)
{
   if(!is_base64(sv)) throw std::runtime_error("input string was not base64");

   auto s = sv.data();

   // Maybe a fraction slower than apache, but I think there code
   // misreports the length.
   size_t n_chars   = 0;
   const uint8_t* p = reinterpret_cast<const uint8_t*>(s);
   while(pr2six[*p++] <= 63) n_chars++;
   size_t rem            = n_chars % 4;
   size_t trailing_bytes = (rem == 2) ? 1 : ((rem == 3) ? 2 : 0);
   return 3 * ((n_chars - rem) / 4) + trailing_bytes;
}

// --------------------------------------------------------------- base64_decode

static size_t base64_decode_(void* dst, const char* s)
{
   uint8_t* write = reinterpret_cast<uint8_t*>(dst);

   // convert to unsigned so can use pr2six
   const uint8_t* p = reinterpret_cast<const uint8_t*>(s);
   int a, b, c, d; // four decoded character values read
   int i = 0;

   size_t sz = base64_decode_length(s);

   // How many groups of 3 bytes are there?
   size_t n_triples = sz / 3;
   for(size_t i = 0; i < n_triples; ++i) {
      a        = pr2six[*p++];
      b        = pr2six[*p++];
      c        = pr2six[*p++];
      d        = pr2six[*p++];
      *write++ = uint8_t(((a << 2) | (b >> 4)) & 0xff);
      *write++ = uint8_t(((b << 4) | (c >> 2)) & 0xff);
      *write++ = uint8_t(((c << 6) | (d >> 0)) & 0xff);
   }

   size_t rem = sz - n_triples * 3;
   if(rem == 0) {
      // nothing to do
   } else if(rem == 1) {
      a        = pr2six[*p++];
      b        = pr2six[*p++];
      *write++ = uint8_t(((a << 2) | (b >> 4)) & 0xff);
   } else if(rem == 2) {
      a        = pr2six[*p++];
      b        = pr2six[*p++];
      c        = pr2six[*p++];
      *write++ = uint8_t(((a << 2) | (b >> 4)) & 0xff);
      *write++ = uint8_t(((b << 4) | (c >> 2)) & 0xff);
   }

   size_t bytes_decoded = size_t(write - reinterpret_cast<const uint8_t*>(dst));
   assert(bytes_decoded == sz);
   assert(pr2six[*p] == 64);

   return bytes_decoded;
}

string base64_decode(const string_view sv) noexcept(false)
{
   const auto sz = base64_decode_length(sv); // may throw
   string o(sz, '\0');
   const auto o_sz = base64_decode_(&o[0], sv.data());
   Expects(o_sz == sz);
   return o;
}

void base64_decode(const std::string_view sv, void* buffer) noexcept(false)
{
   base64_decode_(buffer, sv.data());
}

// -------------------------------------------------------- base64_encode_length

size_t base64_encode_length(size_t sz) noexcept { return (((sz + 2) / 3) * 4); }

// --------------------------------------------------------------- base64_encode

static size_t base64_encode_(char* dst, const void* src, size_t sz) noexcept
{
   char* pos           = dst;
   const uint8_t* read = reinterpret_cast<const uint8_t*>(src);
   uint8_t a, b, c;

   // remainder bytes at the end
   size_t rem = sz % 3;

   // encode all sets of three bytes
   const uint8_t* end = read + sz - rem;
   while(read < end) {
      a      = *read++;
      b      = *read++;
      c      = *read++;
      *pos++ = codebook[(a >> 2) & 0x3f];
      *pos++ = codebook[((a & 0x03) << 4) | (b >> 4)];
      *pos++ = codebook[((b & 0x0f) << 2) | (c >> 6)];
      *pos++ = codebook[c & 0x3f];
   }

   if(rem == 0) {
      // nothing to do
   } else if(rem == 1) {
      a      = *read++;
      *pos++ = codebook[(a >> 2) & 0x3f];
      *pos++ = codebook[((a & 0x03) << 4)];
      *pos++ = '=';
      *pos++ = '=';
   } else if(rem == 2) {
      a      = *read++;
      b      = *read++;
      *pos++ = codebook[(a >> 2) & 0x3f];
      *pos++ = codebook[((a & 0x03) << 4) | (b >> 4)];
      *pos++ = codebook[((b & 0x0f) << 2)];
      *pos++ = '=';
   }

   // Terminate string
   *pos++ = '\0';

   Expects(base64_encode_length(sz) == size_t(pos - dst) - 1);

   return size_t((pos - dst) - 1); // the length
}

string base64_encode(const void* src, size_t sz) noexcept
{
   string s(base64_encode_length(sz) + 1, '\0');
   base64_encode_(&s[0], src, sz);
   Expects(s.back() == '\0');
   s.pop_back();
   return s;
}

string base64_encode(const vector<char>& buf) noexcept
{
   return base64_encode(&buf[0], buf.size());
}

string base64_encode(const string_view buf) noexcept
{
   return base64_encode(buf.data(), buf.size());
}
