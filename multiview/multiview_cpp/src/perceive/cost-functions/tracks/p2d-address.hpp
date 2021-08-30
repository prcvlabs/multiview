
#pragma once

namespace perceive
{
// Should be able to look up a Skeleton detection
struct P2dAddress
{
   int32_t frame_no      = -1;
   uint16_t sensor_no    = uint16_t(-1);
   uint16_t detection_no = uint16_t(-1);

   constexpr P2dAddress(int frame_no_, int sensor_no_, int detection_no_)
       : frame_no(frame_no_)
       , sensor_no(uint16_t(sensor_no_))
       , detection_no(uint16_t(detection_no_))
   {}

   P2dAddress()                  = default;
   P2dAddress(const P2dAddress&) = default;
   P2dAddress(P2dAddress&&)      = default;
   ~P2dAddress()                 = default;
   P2dAddress& operator=(const P2dAddress&) = default;
   P2dAddress& operator=(P2dAddress&&) = default;

   bool operator==(const P2dAddress& o) const noexcept
   {
      return to_u64_() == o.to_u64_();
   }
   bool operator!=(const P2dAddress& o) const noexcept { return !(*this == o); }

   bool operator<(const P2dAddress& o) const noexcept
   {
      return (frame_no != o.frame_no)     ? (frame_no < o.frame_no)
             : (sensor_no != o.sensor_no) ? (sensor_no < o.sensor_no)
                                          : (detection_no < o.detection_no);
   }

   bool is_init() const noexcept { return frame_no >= 0; }
   size_t hash() const noexcept
   {
      std::hash<uint64_t> S;
      return S(to_u64_());
   }

   bool is_interpolation() const noexcept { return detection_no < 0; }

   string to_string() const noexcept
   {
      return format(
          "[{:02d}, {:02d}, {:02d}]", frame_no, sensor_no, detection_no);
   }

 private:
   uint64_t to_u64_() const noexcept
   {
      return (uint64_t(frame_no) << 32) | (uint64_t(sensor_no) << 16)
             | (uint64_t(detection_no) << 0);
   }
};

} // namespace perceive

namespace std
{
// Point4
template<> struct hash<perceive::P2dAddress>
{
   size_t operator()(const perceive::P2dAddress& v) const { return v.hash(); }
};
} // namespace std
