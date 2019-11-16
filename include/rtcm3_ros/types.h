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
  explicit ECEF(const LatLonHeight& llh);
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
  double& x()
  {
    return x_;
  }
  double& y()
  {
    return y_;
  }
  double& z()
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
  explicit LatLonHeight(const ECEF& ecef);
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

ECEF::ECEF(const LatLonHeight& llh)
{
  x_ = llh.height() * cos(llh.lat()) * cos(llh.lon());
  y_ = llh.height() * cos(llh.lat()) * sin(llh.lon());
  z_ = llh.height() * sin(llh.lat());
}
LatLonHeight::LatLonHeight(const ECEF& ecef)
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
  Duration(const double sec)
  {
    *this = Duration(0, static_cast<int64_t>(sec * SEC));
  }
  Duration(const int64_t sec, const int64_t psec)
  {
    if (psec < 0)
    {
      const int64_t carry = psec / SEC;
      sec_ = sec + carry;
      psec_ = psec - SEC * carry;
    }
    else if (psec >= SEC)
    {
      const int64_t carry = psec / SEC;
      sec_ = sec + carry;
      psec_ = psec - SEC * carry;
    }
    else
    {
      sec_ = sec;
      psec_ = psec;
    }
  }
  Duration normalized()
  {
    Duration ret = *this;
    while (ret.sec_ > 302400)
      ret.sec_ -= 604800;
    while (ret.sec_ < -302400)
      ret.sec_ += 604800;

    return ret;
  }
  double toSec() const
  {
    return static_cast<double>(sec_) + psec_ * 1e-12;
  }
  Duration operator+(const Duration& a) const
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
  Duration operator-(const Time& in) const
  {
    return Duration(
        static_cast<int64_t>(sec_) - static_cast<int64_t>(in.sec_),
        static_cast<int64_t>(psec_) - static_cast<int64_t>(in.psec_));
  }
  bool operator==(const Time& in) const
  {
    return sec_ == in.sec_ && psec_ == in.psec_;
  }
  bool operator<(const Time& in) const
  {
    if (sec_ == in.sec_)
      return psec_ < in.psec_;
    return sec_ < in.sec_;
  }
  static Time now()
  {
#ifndef ROSCPP_ROS_H
    timespec start;
    clock_gettime(CLOCK_REALTIME, &start);

    return Time(start.tv_sec, start.tv_nsec * 1000);
#else
    ros::Time rostime = ros::Time::now();
    return Time(rostime.sec, rostime.nsec * 1000);
#endif
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
          std::pair<Time, int>(Time(1483196400), 18),
          std::pair<Time, int>(Time(1435676400), 17),
          std::pair<Time, int>(Time(1341068400), 16),
          std::pair<Time, int>(Time(1230735600), 15),
          std::pair<Time, int>(Time(1136041200), 14),
          std::pair<Time, int>(Time(915116400), 13),
          std::pair<Time, int>(Time(867682800), 12),
          std::pair<Time, int>(Time(820422000), 11),
          std::pair<Time, int>(Time(772988400), 10),
          std::pair<Time, int>(Time(741452400), 9),
          std::pair<Time, int>(Time(709916400), 8),
          std::pair<Time, int>(Time(662655600), 7),
          std::pair<Time, int>(Time(631119600), 6),
          std::pair<Time, int>(Time(567961200), 5),
          std::pair<Time, int>(Time(488991600), 4),
          std::pair<Time, int>(Time(425833200), 3),
          std::pair<Time, int>(Time(394297200), 2),
          std::pair<Time, int>(Time(362761200), 1),
        };
    for (auto& leap : leaps)
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
  Duration operator-(const GTime& in) const
  {
    return Duration(
        week_ - in.week_,
        static_cast<int64_t>(tow_psec_) - static_cast<int64_t>(in.tow_psec_));
  }
  GTime operator+(const Duration& in) const
  {
    return GTime(week_, tow_psec_ + in.sec_ * SEC + in.psec_);
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
  double base_frequency_;
  double wave_length_;

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
      const double doppler_frequency,
      const double base_frequency,
      const double wave_length)
    : time_(time)
    , snr_(snr)
    , lli_(lli)
    , code_(code)
    , pseudo_range_(pseudo_range)
    , phase_(phase)
    , doppler_frequency_(doppler_frequency)
    , base_frequency_(base_frequency)
    , wave_length_(wave_length)
  {
  }
  GTime getTime() const
  {
    return time_;
  }
  double getPseudoRange() const
  {
    return pseudo_range_;
  }
  double getPhaseCycle() const
  {
    return phase_;
  }
  double getDopplerFrequency() const
  {
    return doppler_frequency_;
  }
  double getBaseFrequency() const
  {
    return base_frequency_;
  }
  double getWaveLength() const
  {
    return wave_length_;
  }
  double getSNR() const
  {
    return snr_;
  }
};

class IonoMask
{
protected:
  uint32_t band_;
  uint32_t iodi_;

  std::vector<uint32_t> indice_;

public:
  IonoMask()
    : band_(0)
    , iodi_(-1)
  {
  }
  IonoMask(
      const uint32_t band,
      const uint32_t iodi)
    : band_(band)
    , iodi_(iodi)
  {
  }
  void addMask(const uint32_t id)
  {
    indice_.push_back(id);
  }
  uint32_t getId(const uint32_t num) const
  {
    return indice_[num];
  }
  uint32_t getIodi() const
  {
    return iodi_;
  }
  uint32_t getBand() const
  {
    return band_;
  }
  size_t size() const
  {
    return indice_.size();
  }
};

class IonoDelay
{
protected:
  std::map<std::pair<int, int>, float> delay_;
  std::map<uint32_t, IonoMask> masks_;

  struct YTable
  {
    size_t size_;
    int y_[72];
  };
  const YTable Y_TABLE0 =
      {
        28,
        { -75, -65, -55, -50, -45, -40, -35, -30, -25, -20,
          -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
          35, 40, 45, 50, 55, 65, 75, 85 }
      };
  const YTable Y_TABLE1 =
      {
        23,
        { -55, -50, -45, -40, -35, -30, -25, -20, -15, -10,
          -5, 0, 5, 10, 15, 20, 25, 30, 35, 40,
          45, 50, 55 }
      };
  const YTable Y_TABLE2 =
      {
        27,
        { -75, -65, -55, -50, -45, -40, -35, -30, -25, -20,
          -15, -10, -5, 0, 5, 10, 15, 20, 25, 30,
          35, 40, 45, 50, 55, 65, 75 }
      };
  const YTable Y_TABLE3 =
      {
        28,
        { -85, -75, -65, -55, -50, -45, -40, -35, -30, -25,
          -20, -15, -10, -5, 0, 5, 10, 15, 20, 25,
          30, 35, 40, 45, 50, 55, 65, 75 }
      };
  const YTable Y_TABLE4 =
      {
        72,
        { -180, -175, -170, -165, -160, -155, -150, -145, -140, -135,
          -130, -125, -120, -115, -110, -105, -100, -95, -90, -85,
          -80, -75, -70, -65, -60, -55, -50, -45, -40, -35,
          -30, -25, -20, -15, -10, -5, 0, 5, 10, 15,
          20, 25, 30, 35, 40, 45, 50, 55, 60, 65,
          70, 75, 80, 85, 90, 95, 100, 105, 110, 115,
          120, 125, 130, 135, 140, 145, 150, 155, 160, 165,
          170, 175 }
      };
  const YTable Y_TABLE5 =
      {
        36,
        { -180, -170, -160, -150, -140, -130, -120, -110, -100, -90,
          -80, -70, -60, -50, -40, -30, -20, -10, 0, 10,
          20, 30, 40, 50, 60, 70, 80, 90, 100, 110,
          120, 130, 140, 150, 160, 170 }
      };
  const YTable Y_TABLE6 =
      {
        12,
        { -180, -150, -120, -90, -60, -30, 0, 30, 60, 90,
          120, 150 }
      };
  const YTable Y_TABLE7 =
      {
        12,
        { -170, -140, -110, -80, -50, -20, 10, 40, 70, 100,
          130, 160 }
      };
  struct BandPart
  {
    int x_;
    YTable const* y_;
  };
  struct BandA
  {
    BandPart bands_[8];
  };
  struct BandB
  {
    BandPart bands_[5];
  };
  const BandA BAND_A[9] =
      {
        { { { -180, &Y_TABLE0 },
            { -175, &Y_TABLE1 },
            { -170, &Y_TABLE2 },
            { -165, &Y_TABLE1 },
            { -160, &Y_TABLE2 },
            { -155, &Y_TABLE1 },
            { -150, &Y_TABLE2 },
            { -145, &Y_TABLE1 } } },  // band 0
        { { { -140, &Y_TABLE3 },
            { -135, &Y_TABLE1 },
            { -130, &Y_TABLE2 },
            { -125, &Y_TABLE1 },
            { -120, &Y_TABLE2 },
            { -115, &Y_TABLE1 },
            { -110, &Y_TABLE2 },
            { -105, &Y_TABLE1 } } },  // band1
        { { { -100, &Y_TABLE2 },
            { -95, &Y_TABLE1 },
            { -90, &Y_TABLE0 },
            { -85, &Y_TABLE1 },
            { -80, &Y_TABLE2 },
            { -75, &Y_TABLE1 },
            { -70, &Y_TABLE2 },
            { -65, &Y_TABLE1 } } },  // band2
        { { { -60, &Y_TABLE2 },
            { -55, &Y_TABLE1 },
            { -50, &Y_TABLE3 },
            { -45, &Y_TABLE1 },
            { -40, &Y_TABLE2 },
            { -35, &Y_TABLE1 },
            { -30, &Y_TABLE2 },
            { -25, &Y_TABLE1 } } },  // band3
        { { { -20, &Y_TABLE2 },
            { -15, &Y_TABLE1 },
            { -10, &Y_TABLE2 },
            { -5, &Y_TABLE1 },
            { 0, &Y_TABLE0 },
            { 5, &Y_TABLE1 },
            { 10, &Y_TABLE2 },
            { 15, &Y_TABLE1 } } },  // band4
        { { { 20, &Y_TABLE2 },
            { 25, &Y_TABLE1 },
            { 30, &Y_TABLE2 },
            { 35, &Y_TABLE1 },
            { 40, &Y_TABLE3 },
            { 45, &Y_TABLE1 },
            { 50, &Y_TABLE2 },
            { 55, &Y_TABLE1 } } },  // band5
        { { { 60, &Y_TABLE2 },
            { 65, &Y_TABLE1 },
            { 70, &Y_TABLE2 },
            { 75, &Y_TABLE1 },
            { 80, &Y_TABLE2 },
            { 85, &Y_TABLE1 },
            { 90, &Y_TABLE0 },
            { 95, &Y_TABLE1 } } },  // band6
        { { { 100, &Y_TABLE2 },
            { 105, &Y_TABLE1 },
            { 110, &Y_TABLE2 },
            { 115, &Y_TABLE1 },
            { 120, &Y_TABLE2 },
            { 125, &Y_TABLE1 },
            { 130, &Y_TABLE3 },
            { 135, &Y_TABLE1 } } },  // band7
        { { { 140, &Y_TABLE2 },
            { 145, &Y_TABLE1 },
            { 150, &Y_TABLE2 },
            { 155, &Y_TABLE1 },
            { 160, &Y_TABLE2 },
            { 165, &Y_TABLE1 },
            { 170, &Y_TABLE2 },
            { 175, &Y_TABLE1 } } }  // band8
      };
  const BandB BAND_B[2] =
      {
        { { { 60, &Y_TABLE4 },
            { 65, &Y_TABLE5 },
            { 70, &Y_TABLE5 },
            { 75, &Y_TABLE5 },
            { 85, &Y_TABLE6 } } },  // band 9
        { { { -60, &Y_TABLE4 },
            { -65, &Y_TABLE5 },
            { -70, &Y_TABLE5 },
            { -75, &Y_TABLE5 },
            { -85, &Y_TABLE7 } } }  // band 10
      };

public:
  void updateDelay(
      const uint32_t band,
      const uint32_t num,
      const uint32_t iodi,
      const float delay)
  {
    const auto mask = masks_.find(band);
    if (mask == masks_.end())
      return;
    if (num >= mask->second.size())
      return;
    if (mask->second.getIodi() != iodi)
      return;
    const uint32_t id = mask->second.getId(num);
    int lat, lon;
    if (band < 9)
    {
      size_t grid_start = 0;
      int x(0), y(0);
      for (const auto& b : BAND_A[band].bands_)
      {
        const size_t grid_end = grid_start + b.y_->size_;
        if (id < grid_end)
        {
          x = b.x_;
          y = b.y_->y_[id - grid_start];
          break;
        }
        grid_start = grid_end;
      }
      lon = x;
      lat = y;
    }
    else
    {
      size_t grid_start = 0;
      int x(0), y(0);
      for (const auto& b : BAND_B[band].bands_)
      {
        const size_t grid_end = grid_start + b.y_->size_;
        if (id < grid_end)
        {
          x = b.x_;
          y = b.y_->y_[id - grid_start];
          break;
        }
        grid_start = grid_end;
      }
      lat = x;
      lon = y;
    }
    delay_[std::pair<int, int>(lat, lon)] = delay;
  }
  void updateMask(
      const uint32_t band,
      const IonoMask& mask)
  {
    masks_[band] = mask;
  }
  decltype(delay_)::const_iterator begin() const
  {
    return delay_.begin();
  }
  decltype(delay_)::const_iterator end() const
  {
    return delay_.end();
  }
};
};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_TYPES_H
