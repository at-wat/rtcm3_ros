#ifndef RTCM3_ROS_TYPES_H
#define RTCM3_ROS_TYPES_H

#include <utility>
#include <ctime>

namespace rtcm3_ros
{
class LatLonHeight;
class ECEF;

class GTime;
class Time;

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
  explicit ECEF(const LatLonHeight &llh);
  double x() const
  {
    return x_;
  }
  double y() const
  {
    return y_;
  }
  double z() const
  {
    return z_;
  }
};
class LatLonHeight
{
protected:
  double lat_;
  double lon_;
  double height_;

public:
  LatLonHeight()
    : lat_(0.0)
    , lon_(0.0)
    , height_(0.0)
  {
  }
  LatLonHeight(const double lat, const double lon, const double height)
  {
    lat_ = lat;
    lon_ = lon;
    height_ = height;
  }
  explicit LatLonHeight(const ECEF &ecef);
  double lat() const
  {
    return lat_;
  }
  double lon() const
  {
    return lon_;
  }
  double height() const
  {
    return height_;
  }
};

ECEF::ECEF(const LatLonHeight &llh)
{
  x_ = llh.height() * cos(llh.lat()) * cos(llh.lon());
  y_ = llh.height() * cos(llh.lat()) * sin(llh.lon());
  z_ = llh.height() * sin(llh.lat());
}
LatLonHeight::LatLonHeight(const ECEF &ecef)
{
  lat_ = atan2(ecef.y(), ecef.x());
  lon_ = atan2(ecef.z(), sqrt(pow(ecef.x(), 2) + pow(ecef.y(), 2)));
  height_ = sqrt(pow(ecef.x(), 2) + pow(ecef.y(), 2) + pow(ecef.z(), 2));
}

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
  double toSec() const
  {
    return static_cast<double>(sec_) + psec_ * 1e-12;
  }
  Duration operator+(const Duration &a) const
  {
    return Duration(sec_ + a.sec_, psec_ + a.psec_);
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
  explicit Time(const uint64_t sec)
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
  GTime(uint64_t week, uint64_t tow_psec)
  {
    week_ = week;
    tow_psec_ = tow_psec;
  }
  explicit GTime(const Time t)
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
  static GTime fromTow(const double tow, const Time t0 = Time::now())
  {
    return fromTow(static_cast<uint64_t>(tow * SEC), t0);
  }
  static GTime fromTow(const uint64_t tow_psec, const Time t0 = Time::now())
  {
    GTime ret;
    GTime time(t0);
    ret.week_ = time.week_;
    ret.tow_psec_ = tow_psec;

    return ret;
  }
  static GTime fromTow(const uint64_t tow_psec, const GTime t0)
  {
    GTime ret;
    ret.week_ = t0.week_;
    ret.tow_psec_ = tow_psec;

    return ret;
  }
  static GTime from10bitsWeek(const uint64_t week)
  {
    GTime now = GTime(Time::now());
    if (now.week_ < 1560)
      now.week_ = 1560;

    return GTime(week + (now.week_ - week + 512) / 1024 * 1024, 0);
  }
  double getTow() const
  {
    return static_cast<double>(tow_psec_) / static_cast<double>(SEC);
  }
  uint64_t getWeek() const
  {
    return week_;
  }
  Duration operator-(const GTime &in) const
  {
    return Duration(
        week_ - in.week_,
        static_cast<int64_t>(tow_psec_) - static_cast<int64_t>(in.tow_psec_));
  }
};

class Range
{
protected:
  GTime time_;
  double snr_;
  int8_t lli_;
  int8_t code_;
  double phase_;
  double pseudo_range_;
  double doppler_frequency_;

public:
  Range()
  {
  }
  Range(
      const GTime time,
      const double snr,
      const int8_t lli,
      const int8_t code,
      const double pseudo_range,
      const double phase,
      const double doppler_frequency)
  {
    time_ = time;
    snr_ = snr;
    lli_ = lli;
    code_ = code;
    phase_ = phase;
    pseudo_range_ = pseudo_range;
    doppler_frequency_ = doppler_frequency;
  }
  GTime getTime() const
  {
    return time_;
  }
  double getPseudoRange() const
  {
    return pseudo_range_;
  }
  double getPhaseRange() const
  {
    return phase_;
  }
  double getDopplerFrequency() const
  {
    return doppler_frequency_;
  }
};

};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_TYPES_H
