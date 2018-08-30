#ifndef RTCM3_ROS_MESSAGES_RTCM3_MESSAGES_BASE_H
#define RTCM3_ROS_MESSAGES_RTCM3_MESSAGES_BASE_H

#include <map>

#include <rtcm3_ros/types.h>
#include <rtcm3_ros/constants.h>
#include <rtcm3_ros/buffer.h>

namespace rtcm3_ros
{
class RTCM3MessageBase
{
public:
  using Ptr = std::shared_ptr<RTCM3MessageBase>;
  enum Category
  {
    CORRECTION_CLOCK,
    CORRECTION_ORBIT,
    EPHEMERIDES,
    PSEUDO_RANGE,
    SBAS
  };
  virtual int getType() const = 0;
  virtual int getCategory() const = 0;
  virtual bool decode(const Buffer &) = 0;
};

class RTCM3MessageEphemeridesBase : public RTCM3MessageBase
{
public:
  using Ptr = std::shared_ptr<RTCM3MessageEphemeridesBase>;
  int getCategory() const
  {
    return Category::EPHEMERIDES;
  }
  virtual bool decode(const Buffer &) = 0;
  virtual double getClockBias(const GTime &time) const = 0;
  virtual ECEF getPos(const GTime &time) const = 0;
  virtual int getSatId() const = 0;
};

class RTCM3MessagePseudoRangeBase : public RTCM3MessageBase
{
public:
  using Ptr = std::shared_ptr<RTCM3MessagePseudoRangeBase>;
  using Container = std::map<int, Range>;

  int getCategory() const
  {
    return Category::PSEUDO_RANGE;
  }
  virtual bool decode(const Buffer &) = 0;
  virtual Container::iterator begin() = 0;
  virtual Container::iterator end() = 0;
  const Container::iterator begin() const
  {
    return begin();
  }
  const Container::iterator end() const
  {
    return end();
  }
};

static constexpr double UPDATE_INTERVAL[16] = {
  1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600, 900, 1800, 3600, 7200, 10800
};

class RTCM3MessageCorrectionsBase : public RTCM3MessageBase
{
public:
  using Ptr = std::shared_ptr<RTCM3MessageCorrectionsBase>;
  virtual bool decode(const Buffer &) = 0;
};

};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_MESSAGES_RTCM3_MESSAGES_BASE_H
