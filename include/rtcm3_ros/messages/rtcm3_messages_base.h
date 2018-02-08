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
  virtual int getType() const = 0;
  virtual bool decode(const Buffer &) = 0;
};

class RTCM3MessageEphemeridesBase : public RTCM3MessageBase
{
public:
  virtual int getType() const = 0;
  virtual bool decode(const Buffer &) = 0;
  virtual ECEF getPos(const int &sat, const GTime &time) const = 0;
};

class RTCM3MessagePseudoRangeBase : public RTCM3MessageBase
{
protected:
  int station_;

public:
  virtual int getType() const = 0;
  virtual bool decode(const Buffer &) = 0;
};

};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_MESSAGES_RTCM3_MESSAGES_BASE_H
