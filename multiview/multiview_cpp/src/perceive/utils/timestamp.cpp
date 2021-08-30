
#include "timestamp.hpp"

#include <limits.h>
#include <regex>

#define This Timestamp

namespace perceive
{
// Doesn't count 'year' in the number of leap years
static int n_leap_years_to_epoch(int year) noexcept
{
   int div_4s   = 0;
   int div_100s = 0;
   int div_400s = 0;

   if(year >= 1972) {
      div_4s = ((year - 1972) / 4) + 1;
   } else if(year <= 1967) {
      div_4s = ((1968 - year) / 4) + 1;
   }

   if(year >= 2000) {
      div_100s = ((year - 2000) / 100) + 1;
   } else if(year <= 1900) {
      div_100s = ((1900 - year) / 100) + 1;
   }

   if(year >= 2000) {
      div_400s = ((year - 2000) / 400) + 1;
   } else if(year <= 1600) {
      div_400s = ((1600 - year) / 400) + 1;
   }

   return div_4s - div_100s + div_400s - (is_leap_year(year) ? 1 : 0);
}

// ---------------------------------------------------------------- is leap year

bool is_leap_year(int y) noexcept
{
   return (y % 4 == 0) and ((y % 100 != 0) or (y % 400 == 0));
}

// --------------------------------------------------------------- is valid date

bool is_valid_date(int y, int m, int d) noexcept
{
   if(m < 1 or m > 12) return false;
   if(d < 1 or d > 31) return false;
   if(d > 30 and (m == 4 or m == 6 or m == 9 or m == 11)) return false;
   if(m == 2) {
      if(d > 29) return false;
      if(d == 29 and not is_leap_year(y)) return false;
   }
   return true;
}

// --------------------------------------------------------------- is valid time

bool is_valid_time(int h, int m, int s) noexcept
{
   return (h >= 0 and h < 24) and (m >= 0 and m < 60) and (s >= 0 and s < 60);
}

// ----------------------------------------------------------------- day of week

int day_of_week(int y, int m, int d) noexcept
{
   m -= 2;
   if(m < 1) {
      m += 12; // March is 1, Feb is 12
      y -= 1;
   }

   int D = y % 100;
   int C = y / 100;
   int f = d + (13 * m - 1) / 5 + D + D / 4 + C / 4 - 2 * C;
   int w = f % 7;
   if(w < 0) w += 7;
   return w;
}

// ----------------------------------------------------------------- day of year

int day_of_year(int y, int m, int d) noexcept
{
   auto ret = (d - 1) + (m > 2 and is_leap_year(y) ? 1 : 0);
   static constexpr array<int, 12> L{
       {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334}};
   if(m >= 1 and m <= 12) ret += L[size_t(m - 1)];
   return ret;
}

// --------------------------------------------------------------------- str(tm)

std::string str(const std::tm& tm)
{
   std::stringstream ss("");
   ss << format("Y = {}", tm.tm_year + 1900) << std::endl;
   ss << format("m = {}", tm.tm_mon + 1) << std::endl;
   ss << format("d = {}", tm.tm_mday) << std::endl;
   ss << format("H = {}", tm.tm_hour) << std::endl;
   ss << format("M = {}", tm.tm_min) << std::endl;
   ss << format("S = {}", tm.tm_sec) << std::endl;
   ss << format("dy= {}", tm.tm_yday) << std::endl;
   ss << format("wy= {}", tm.tm_wday) << std::endl;
   ss << format("_ = {}", tm.tm_isdst) << std::endl;
   return ss.str();
}

// --------------------------------------------------------------- Seconds to TM
// This function comes from MUSL libc, and is under the MIT license
bool seconds_to_tm(int64_t t, std::tm* tm) noexcept
{
   // 2000-03-01 (mod 400 year, immediately after feb29
   constexpr int64_t LEAPOCH       = (946684800LL + 86400 * (31 + 29));
   constexpr int64_t DAYS_PER_400Y = (365 * 400 + 97);
   constexpr int64_t DAYS_PER_100Y = (365 * 100 + 24);
   constexpr int64_t DAYS_PER_4Y   = (365 * 4 + 1);

   long long days, secs;
   int remdays, remsecs, remyears;
   int qc_cycles, c_cycles, q_cycles;
   int years, months;
   int wday, yday, leap;
   static const char days_in_month[]
       = {31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 29};

   // Reject time_t values whose year would overflow int
   if(t < INT_MIN * 31622400LL || t > INT_MAX * 31622400LL) return false;

   secs    = t - LEAPOCH;
   days    = secs / 86400;
   remsecs = secs % 86400;
   if(remsecs < 0) {
      remsecs += 86400;
      days--;
   }

   wday = (3 + days) % 7;
   if(wday < 0) wday += 7;

   qc_cycles = int(days / DAYS_PER_400Y);
   remdays   = days % DAYS_PER_400Y;
   if(remdays < 0) {
      remdays += DAYS_PER_400Y;
      qc_cycles--;
   }

   c_cycles = remdays / DAYS_PER_100Y;
   if(c_cycles == 4) c_cycles--;
   remdays -= c_cycles * DAYS_PER_100Y;

   q_cycles = remdays / DAYS_PER_4Y;
   if(q_cycles == 25) q_cycles--;
   remdays -= q_cycles * DAYS_PER_4Y;

   remyears = remdays / 365;
   if(remyears == 4) remyears--;
   remdays -= remyears * 365;

   leap = !remyears and (q_cycles or !c_cycles);
   yday = remdays + 31 + 28 + leap;
   if(yday >= 365 + leap) yday -= 365 + leap;

   years = remyears + 4 * q_cycles + 100 * c_cycles + 400 * qc_cycles;

   for(months = 0; days_in_month[months] <= remdays; months++)
      remdays -= days_in_month[months];

   if(years + 100 > INT_MAX || years + 100 < INT_MIN) return false;

   tm->tm_year = years + 100;
   tm->tm_mon  = months + 2;
   if(tm->tm_mon >= 12) {
      tm->tm_mon -= 12;
      tm->tm_year++;
   }
   tm->tm_mday = remdays + 1;
   tm->tm_wday = wday;
   tm->tm_yday = yday;

   tm->tm_hour = remsecs / 3600;
   tm->tm_min  = remsecs / 60 % 60;
   tm->tm_sec  = remsecs % 60;

   return true;
}

// ---------------------------------------------------------------- year to secs
// This function comes from MUSL libc, and is under the MIT license
static int64_t year_to_secs(int64_t year, int* is_leap)
{
   //   Expects(is_leap != nullptr);

   if(uint64_t(year) - 2ULL <= 136) {
      int y     = int(year);
      int leaps = (y - 68) >> 2;
      if(!((y - 68) & 3)) {
         leaps--;
         if(is_leap) *is_leap = 1;
      } else if(is_leap)
         *is_leap = 0;
      return 31536000 * (y - 70) + 86400 * leaps;
   }

   int cycles, centuries, leaps, rem;

   cycles = int((year - 100) / 400);
   rem    = (year - 100) % 400;
   if(rem < 0) {
      cycles--;
      rem += 400;
   }
   if(!rem) {
      *is_leap  = 1;
      centuries = 0;
      leaps     = 0;
   } else {
      if(rem >= 200) {
         if(rem >= 300)
            centuries = 3, rem -= 300;
         else
            centuries = 2, rem -= 200;
      } else {
         if(rem >= 100)
            centuries = 1, rem -= 100;
         else
            centuries = 0;
      }
      if(!rem) {
         *is_leap = 0;
         leaps    = 0;
      } else {
         leaps = unsigned(rem) / 4U;
         rem %= 4U;
         *is_leap = !rem;
      }
   }

   leaps += 97 * cycles + 24 * centuries - *is_leap;

   return (year - 100) * 31536000LL + leaps * 86400LL + 946684800 + 86400;
}

// --------------------------------------------------------- datetime to seconds
// This function comes from MUSL libc, and is under the MIT license
int64_t datetime_to_seconds(int y, int m, int d, int H, int M, int S) noexcept
{
   int is_leap;
   int year  = y - 1900;
   int month = m - 1;
   if(month >= 12 || month < 0) {
      int adj = month / 12;
      month %= 12;
      if(month < 0) {
         adj--;
         month += 12;
      }
      year += adj;
   }
   int64_t t = year_to_secs(year, &is_leap);
   t += day_of_year(y, m, d) * 86400LL;
   // t += __month_to_secs(month, is_leap);
   // t += 86400LL * (m - 1);
   t += 3600LL * H;
   t += 60LL * M;
   t += S;
   return t;
}

// ------------------------------------------------------------------- To String

std::string This::to_string() const noexcept
{
   std::tm tm = to_tm();
   return format("{:04d}-{:02d}-{:02d}T{:02d}:{:02d}:{:02d}.{:06d}",
                 tm.tm_year + 1900,
                 tm.tm_mon + 1,
                 tm.tm_mday,
                 tm.tm_hour,
                 tm.tm_min,
                 tm.tm_sec,
                 micros());
}

// ---------------------------------------------------------------------- Unpack

void This::unpack(int& y,
                  int& m,
                  int& d,
                  int& hour,
                  int& min,
                  int& sec,
                  int& micros) const noexcept
{
   std::tm tm = to_tm();
   y          = tm.tm_year + 1900;
   m          = tm.tm_mon + 1;
   d          = tm.tm_mday;
   hour       = tm.tm_hour;
   min        = tm.tm_min;
   sec        = tm.tm_sec;
   micros     = int(this->micros());
}

// ------------------------------------------------------------------------- Set

void This::set(int y,
               int m,
               int d,
               int hour,
               int min,
               int sec,
               int micros) noexcept(false)
{
   if(!is_valid_date(y, m, d))
      throw std::runtime_error(
          format("invalid date: {:04d}-{:02d}-{:02d}", y, m, d));
   if(!is_valid_time(hour, min, sec))
      throw std::runtime_error(
          format("invalid time: {:02d}:{:02d}:{:02d}", hour, min, sec));
   if(micros < 0 or micros > 999999)
      throw std::runtime_error(format("invalid microseconds: {}", micros));

   const auto epoch = datetime_to_seconds(y, m, d, hour, min, sec);
   set(epoch, micros);
}

// ------------------------------------------------------------------------- Set

void This::set(const system_clock_time_point& when) noexcept
{
   // Want seconds and micros since epoch
   const auto duration = when.time_since_epoch();
   const int64_t seconds
       = std::chrono::floor<std::chrono::seconds>(duration).count();
   const int micros
       = std::chrono::duration_cast<std::chrono::microseconds>(duration).count()
         % 1000000;
   set(seconds, micros);
}

// ----------------------------------------------------------------------- Parse

Timestamp This::parse(const std::string_view s) noexcept(false)
{
   // YYYY-MM-DDtHH:MM:SS.micros+0000

   // 0123456789.123456789.123456789.
   // 2018-04-11t21-35-56.356193+0000
   // 2018-04-11T21:35:56.356193+0000

   if(s.size() < 10 or s.size() > 31) {
      throw std::runtime_error(format("parse error: timestamp string "
                                      "'{}' length is {}, but must be "
                                      "10-31 characters long",
                                      s,
                                      s.size()));
   }

   // Sanity checks
   for(auto i = 0u; i < s.size(); ++i) {
      if(i == 4 or i == 7) {
         if(s[i] != '-')
            throw std::runtime_error(format("unexpected character"));
      } else if(i == 13 or i == 16) {
         if(s[i] != '-' and s[i] != ':')
            throw std::runtime_error(format("unexpected character"));
      } else if(i == 10) {
         if(s[i] != 't' and s[i] != 'T')
            throw std::runtime_error(format("unexpected character"));
      } else if(i == 19) {
         if(s[i] != '.')
            throw std::runtime_error(format("unexpected character"));
      } else if(i == 26) {
         if(s[i] != '+')
            throw std::runtime_error(format("unexpected character"));
      } else if(!std::isdigit(s[i])) {
         throw std::runtime_error(format("parse error: unexpected "
                                         "character '{:c}' at index "
                                         "{}",
                                         s[i],
                                         i));
      }
   }

   auto d = [&](unsigned ind) -> int {
      return (ind < s.size()) ? int(s[ind] - '0') : 0;
   };

   int year   = d(0) * 1000 + d(1) * 100 + d(2) * 10 + d(3);
   int month  = d(5) * 10 + d(6);
   int day    = d(8) * 10 + d(9);
   int hour   = d(11) * 10 + d(12);
   int min    = d(14) * 10 + d(15);
   int sec    = d(17) * 10 + d(18);
   int micros = d(20) * 100000 + d(21) * 10000 + d(22) * 1000 + d(23) * 100
                + d(24) * 10 + d(25);

   return Timestamp(year, month, day, hour, min, sec, micros);
}

// ------------------------------------------------------------------------- now
//
Timestamp Timestamp::now() noexcept
{
   return Timestamp(std::chrono::system_clock::now());
}

// ----------------------------------------------------------- extract-timestamp
//
string extract_timestamp(const string_view sv) noexcept
{
   static const std::regex re = std::regex("([0-9tT\\:\\-\\+\\.]+)");

   std::string s = sv.data();
   std::reverse(begin(s), end(s));
   std::smatch match;
   while(std::regex_search(s, match, re)) {
      std::string m = match.str();
      try {
         std::reverse(begin(m), end(m));
         Timestamp::parse(m);
         return m;
      } catch(...) {}
      s = match.suffix();
   }

   return "";
}

} // namespace perceive
