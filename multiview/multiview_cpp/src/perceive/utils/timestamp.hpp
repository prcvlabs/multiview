
#pragma once

#include <chrono>
#include <ctime>
#include <functional>
#include <iostream>
#include <stdint.h>
#include <string>

namespace perceive
{
// Some useful functions to do with dates
bool is_leap_year(int y) noexcept;
bool is_valid_date(int y, int m, int d) noexcept;
bool is_valid_time(int h, int m, int s) noexcept;
int day_of_week(int y, int m, int d) noexcept;
int day_of_year(int y, int m, int d) noexcept;
bool seconds_to_tm(int64_t t, std::tm* tm) noexcept;
int64_t datetime_to_seconds(int y, int m, int d, int H, int M, int S) noexcept;

struct Timestamp
{
 public:
   using value_type              = __int128_t;
   using system_clock_time_point = decltype(std::chrono::system_clock::now());

 private:
   static constexpr value_type M = 1000000;
   value_type x{0};

 public:
   Timestamp() = default;
   Timestamp(int64_t t, int micros = 0) { set(t, micros); }
   Timestamp(const std::string_view s) { *this = parse(s); }
   Timestamp(int y,
             int m,
             int d,
             int hour   = 0,
             int min    = 0,
             int sec    = 0,
             int micros = 0) noexcept(false)
   {
      set(y, m, d, hour, min, sec, micros);
   }
   Timestamp(const system_clock_time_point& when) noexcept { set(when); }
   Timestamp(const Timestamp&) = default;
   Timestamp(Timestamp&&)      = default;
   ~Timestamp()                = default;
   Timestamp& operator=(const Timestamp&) = default;
   Timestamp& operator=(Timestamp&&) = default;

   value_type raw() const noexcept { return x; }
   bool empty() const noexcept { return x == 0; }

   static Timestamp parse(const std::string_view s) noexcept(false);
   static Timestamp now() noexcept;

   bool operator==(const Timestamp& o) const noexcept { return x == o.x; }
   bool operator!=(const Timestamp& o) const noexcept { return x != o.x; }
   bool operator<(const Timestamp& o) const noexcept { return x < o.x; }
   bool operator<=(const Timestamp& o) const noexcept { return x <= o.x; }
   bool operator>(const Timestamp& o) const noexcept { return x > o.x; }
   bool operator>=(const Timestamp& o) const noexcept { return x >= o.x; }

   // Adds/subtracts a microsecond
   Timestamp operator++(int) const
   {
      Timestamp t(*this);
      ++t.x;
      return t;
   }
   Timestamp operator--(int) const
   {
      Timestamp t(*this);
      --t.x;
      return t;
   }
   Timestamp& operator++()
   {
      ++x;
      return *this;
   }
   Timestamp& operator--()
   {
      --x;
      return *this;
   }

   // Adds/subtracts microseconds
   Timestamp& operator+=(int64_t v) noexcept
   {
      x += v;
      return *this;
   }
   Timestamp& operator-=(int64_t v) noexcept
   {
      x -= v;
      return *this;
   }
   Timestamp operator+(int64_t v) noexcept
   {
      Timestamp t(*this);
      t += v;
      return t;
   }
   Timestamp operator-(int64_t v) noexcept
   {
      Timestamp t(*this);
      t -= v;
      return t;
   }

   // Adds/subtracts seconds
   Timestamp& add_seconds(double v) noexcept
   {
      x += value_type(v * 1000000.0);
      return *this;
   }

   Timestamp& add_micros(int64_t v) noexcept
   {
      x += v;
      return *this;
   }
   Timestamp& add_millis(int64_t v) noexcept { return add_micros(v * 1000); }
   Timestamp& add_seconds(int64_t v) noexcept
   {
      return add_micros(v * 1000000);
   }
   Timestamp& add_minutes(int64_t v) noexcept { return add_seconds(v * 60); }
   Timestamp& add_hours(int64_t v) noexcept { return add_seconds(v * 3600); }
   Timestamp& add_days(int64_t v) noexcept { return add_seconds(v * 86400); }

   // Distances
   value_type micros_to(const Timestamp& v) const noexcept { return v.x - x; }
   double seconds_to(const Timestamp& v) const noexcept
   {
      value_type diff   = micros_to(v);
      value_type s_part = diff / M;
      value_type m_part = diff % M;
      return double(s_part) + double(m_part) * 1e-6;
   }

   // Getters/setters
   uint32_t micros() const noexcept { return uint32_t(x % M); }
   int64_t seconds_from_epoch() const noexcept { return int64_t(x / M); }

   void set_micros(int micros) noexcept
   {
      x = value_type(seconds_from_epoch()) * M + value_type(micros);
   }
   void set_seconds_to_epoch(int64_t s) noexcept
   {
      x = value_type(s) * M + value_type(micros());
   }
   void set(int64_t s, int micros) noexcept
   {
      set_seconds_to_epoch(s);
      set_micros(micros);
   }

   void set(int y,
            int m,
            int d,
            int hour   = 0,
            int min    = 0,
            int sec    = 0,
            int micros = 0) noexcept(false);

   void set(const system_clock_time_point& when) noexcept;

   std::tm to_tm() const noexcept
   {
      std::tm tm;
      seconds_to_tm(seconds_from_epoch(), &tm);
      return tm;
   }

   void unpack(int& y,
               int& m,
               int& d,
               int& hour,
               int& min,
               int& sec,
               int& micros) const noexcept;

   int year() const noexcept
   {
      int y, m, d, hour, min, sec, micros;
      unpack(y, m, d, hour, min, sec, micros);
      return y;
   }

   int month() const noexcept
   {
      int y, m, d, hour, min, sec, micros;
      unpack(y, m, d, hour, min, sec, micros);
      return m;
   }

   int day() const noexcept
   {
      int y, m, d, hour, min, sec, micros;
      unpack(y, m, d, hour, min, sec, micros);
      return d;
   }

   int hour() const noexcept
   {
      int y, m, d, hour, min, sec, micros;
      unpack(y, m, d, hour, min, sec, micros);
      return hour;
   }

   int minute() const noexcept
   {
      int y, m, d, hour, min, sec, micros;
      unpack(y, m, d, hour, min, sec, micros);
      return min;
   }

   int second() const noexcept
   {
      int y, m, d, hour, min, sec, micros;
      unpack(y, m, d, hour, min, sec, micros);
      return sec;
   }

   std::string to_string() const noexcept;

   friend std::string str(const Timestamp& x) { return x.to_string(); }
};

std::string str(const std::tm& tm);

inline double distance(const Timestamp& a, const Timestamp& b) noexcept
{
   return a.seconds_to(b);
}

inline Timestamp add_seconds(const Timestamp& t, double s) noexcept
{
   Timestamp r(t);
   r.add_seconds(s);
   return r;
}

// Searches 'str' for a parsable timestamp.
// Returns first found, searching from the right, or "" if none found
std::string extract_timestamp(const std::string_view str) noexcept;

} // namespace perceive
