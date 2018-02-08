#ifndef RTCM3_ROS_TYPES_H
#define RTCM3_ROS_TYPES_H

#include <ctime>

namespace rtcm3_ros
{
class ECEF
{
protected:
  double x_;
  double y_;
  double z_;

public:
  ECEF()
    : x_(0.0)
    , y_(0.0)
    , z_(0.0)
  {
  }
  ECEF(const double x, const double y, const double z)
    : x_(x)
    , y_(y)
    , z_(z)
  {
  }
};

class GTime;
class Time;

class Duration
{
protected:
  int64_t sec_;
  int64_t psec_;
  static const int64_t SEC = 1000000000000;

public:
  friend GTime;

  Duration()
  {
    sec_ = psec_ = 0;
  }
  Duration(int64_t sec, int64_t psec)
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

class Time
{
protected:
  uint64_t sec_;
  uint64_t psec_;
  static const int64_t SEC = 1000000000000;

public:
  friend GTime;

  Time()
  {
    sec_ = psec_ = 0;
  }
  Time(const uint64_t sec)
  {
    sec_ = sec;
    psec_ = 0;
  }
  Time(const uint64_t sec, const uint64_t psec)
  {
    sec_ = sec;
    psec_ = psec;
  }
  Duration operator-(const Time &in) const
  {
    return Duration(
        static_cast<int64_t>(sec_) - static_cast<int64_t>(in.sec_),
        static_cast<int64_t>(psec_) - static_cast<int64_t>(in.psec_));
  }
  bool operator==(const Time &in) const
  {
    return sec_ == in.sec_ && psec_ == in.psec_;
  }
  bool operator<(const Time &in) const
  {
    if (sec_ == in.sec_)
      return psec_ < in.psec_;
    return sec_ < in.sec_;
  }
  double toSec() const
  {
    return static_cast<double>(sec_) + psec_ * 1e-12;
  }
  static Time now()
  {
    timespec start;
    clock_gettime(CLOCK_REALTIME, &start);

    Time now;
    now.sec_ = start.tv_sec;
    now.psec_ = start.tv_nsec * 1000;
    return now;
  }
};
class GTime
{
protected:
  uint64_t week_;
  uint64_t tow_psec_;
  static const int64_t SEC = 1000000000000;

public:
  GTime()
  {
    week_ = tow_psec_ = 0;
  }
  GTime(const Time t)
  {
    Time t2 = t;
    const std::pair<Time, int> leaps[] =
        {
          std::pair<Time, int>(Time(1483196400), -18),
          std::pair<Time, int>(Time(1435676400), -17),
          std::pair<Time, int>(Time(1341068400), -16),
          std::pair<Time, int>(Time(1230735600), -15),
          std::pair<Time, int>(Time(1136041200), -14),
          std::pair<Time, int>(Time(915116400), -13),
          std::pair<Time, int>(Time(867682800), -12),
          std::pair<Time, int>(Time(820422000), -11),
          std::pair<Time, int>(Time(772988400), -10),
          std::pair<Time, int>(Time(741452400), -9),
          std::pair<Time, int>(Time(709916400), -8),
          std::pair<Time, int>(Time(662655600), -7),
          std::pair<Time, int>(Time(631119600), -6),
          std::pair<Time, int>(Time(567961200), -5),
          std::pair<Time, int>(Time(488991600), -4),
          std::pair<Time, int>(Time(425833200), -3),
          std::pair<Time, int>(Time(394297200), -2),
          std::pair<Time, int>(Time(362761200), -1),
        };
    for (auto &leap : leaps)
    {
      if (leap.first < t2)
      {
        t2.sec_ += leap.second;
        break;
      }
    }
    const Time gps_time_epoch(315932400);
    Duration duration = t2 - gps_time_epoch;
    week_ = duration.sec_ / (86400 * 7);
    tow_psec_ = (duration.sec_ - week_ * (86400 * 7)) * SEC +
                duration.psec_;
  }
  static GTime fromTow(const uint64_t tow_psec, const Time t0 = Time::now())
  {
    GTime ret;
    GTime time(t0);
    ret.week_ = time.week_;
    ret.tow_psec_ = tow_psec;
    if (ret.tow_psec_ < time.tow_psec_ - 302400 * SEC)
      ret.tow_psec_ += 604800 * SEC;
    else if (ret.tow_psec_ > time.tow_psec_ + 302400 * SEC)
      ret.tow_psec_ -= 604800 * SEC;

    return ret;
  }
};

class PsudoRange
{
protected:
  Time time_;
  float snr_;
  int8_t lli_;
  int8_t code_;
  double phase_;
  double psudo_range_;
  double doppler_frequency_;

public:
  PsudoRange(
      const Time time,
      const float snr,
      const int8_t lli,
      const int8_t code,
      const double phase,
      const double psudo_range,
      const double doppler_frequency)
  {
    time_ = time;
    snr_ = snr;
    lli_ = lli;
    code_ = code;
    phase_ = phase;
    psudo_range_ = psudo_range;
    doppler_frequency_ = doppler_frequency;
  }
};
};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_TYPES_H
