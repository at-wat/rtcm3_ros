#ifndef RTCM3_ROS_MESSAGES_CORRECTIONS_H
#define RTCM3_ROS_MESSAGES_CORRECTIONS_H

#include <rtcm3_ros/messages/rtcm3_messages_base.h>

// 1057, 1058
namespace rtcm3_ros
{
class RTCM3MessageCorrectionsOrbit : public RTCM3MessageCorrectionsBase
{
protected:
  virtual int getNumSatBits() const = 0;

public:
  constexpr int getType() const
  {
    return 1057;
  }
  bool decode(const Buffer &buf)
  {
    size_t i = 24 + 12;
    const GTime now = GTime(Time::now());
    double tow = buf.getUnsignedBits(i, 20) * 1.0;
    if (tow < now.getTow() - 302400.0)
      tow += 604800.0;
    else if (tow > now.getTow() + 302400.0)
      tow -= 604800.0;
    const GTime stamp = GTime::fromTow(tow);
    i += 20;

    double update_interval = UPDATE_INTERVAL[buf.getUnsignedBits(i, 4)];
    i += 4;
    unsigned int sync = buf.getUnsignedBits(i, 1);
    i += 1;
    unsigned int refd = buf.getUnsignedBits(i, 1);
    i += 1;
    unsigned int iod = buf.getUnsignedBits(i, 4);
    i += 4;
    unsigned int provider_id = buf.getUnsignedBits(i, 16);
    i += 16;
    unsigned int solution_id = buf.getUnsignedBits(i, 4);
    i += 4;
    unsigned int nsats = buf.getUnsignedBits(i, getNumSatBits());
    i += getNumSatBits();

    ROS_DEBUG(" - prov_id: %d, sol_id: %d, sats: %d",
              provider_id, solution_id,
              nsats);

    return true;
  }
  double getClockBias(const GTime &time) const
  {
  }
};
class RTCM3MessageCorrectionsOrbitGPS : public RTCM3MessageCorrectionsOrbit
{
protected:
  constexpr int getNumSatBits() const
  {
    return 6;
  }
};
}  // namespace rtcm3_ros

#endif  // RTCM3_ROS_MESSAGES_CORRECTIONS_H
