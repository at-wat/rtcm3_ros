#ifndef RTCM3_ROS_TYPES_H
#define RTCM3_ROS_TYPES_H

namespace rtcm3_ros
{
class ECFC
{
protected:
  double x_;
  double y_;
  double z_;

public:
  ECFC()
    : x_(0.0)
    , y_(0.0)
    , z_(0.0)
  {
  }
  ECFC(const double x, const double y, const double z)
    : x_(x)
    , y_(y)
    , z_(z)
  {
  }
};

class GDuration
{
protected:
  int64_t sec_;
  int64_t psec_;
  const int64_t SEC = 1000000000000;

public:
  GDuration()
  {
    sec_ = psec_ = 0;
  }
  GDuration(int64_t sec, int64_t psec)
  {
    if (psec < 0)
    {
      const int carry = static_cast<int64_t>(-psec / SEC);
      sec_ = sec - carry;
      psec_ = psec + SEC * carry;
    }
    else if (psec >= SEC)
    {
      const int carry = static_cast<int64_t>(-psec / SEC);
      sec_ = sec + carry;
      psec_ = psec - SEC * carry;
    }
    else
    {
      sec_ = sec;
      psec_ = psec;
    }
  }
};
class GTime
{
protected:
  uint64_t sec_;
  uint64_t psec_;

public:
  GTime()
  {
    sec_ = psec_ = 0;
  }
  GTime(uint64_t sec, uint64_t psec)
  {
    sec_ = sec;
    psec_ = psec;
  }
  GDuration operator-(const GTime &in) const
  {
    return GDuration(
        static_cast<int64_t>(sec_) - static_cast<int64_t>(in.sec_),
        static_cast<int64_t>(psec_) - static_cast<int64_t>(in.psec_));
  }
};
};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_TYPES_H
