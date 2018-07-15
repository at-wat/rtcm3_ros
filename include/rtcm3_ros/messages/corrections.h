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
  virtual int getIodeBits() const = 0;
  virtual int getIodcrcBits() const = 0;
  virtual int getPrnOffset() const = 0;

  double update_interval_;
  unsigned int sync_;
  unsigned int refd_;
  unsigned int iod_;
  unsigned int provider_id_;
  unsigned int solution_id_;

  GTime stamp_;

  class OrbitCorrection
  {
  public:
    unsigned int sat_id_;
    unsigned int iode_;
    unsigned int iodcrc_;
    double deph0_;
    double deph1_;
    double deph2_;
    double ddeph0_;
    double ddeph1_;
    double ddeph2_;

    OrbitCorrection()
    {
    }
    OrbitCorrection(
        const unsigned int sat_id,
        const unsigned int iode,
        const unsigned int iodcrc,
        const double deph0,
        const double deph1,
        const double deph2,
        const double ddeph0,
        const double ddeph1,
        const double ddeph2)
      : sat_id_(sat_id)
      , iode_(iode)
      , iodcrc_(iodcrc)
      , deph0_(deph0)
      , deph1_(deph1)
      , deph2_(deph2)
      , ddeph0_(ddeph0)
      , ddeph1_(ddeph1)
      , ddeph2_(ddeph2)
    {
    }
  };
  std::map<size_t, OrbitCorrection> corrections_;

public:
  using Ptr = std::shared_ptr<RTCM3MessageCorrectionsOrbit>;
  int getCategory() const
  {
    return Category::CORRECTION_ORBIT;
  }
  bool decode(const Buffer &buf)
  {
    size_t i = 24 + 12;
    double tow = buf.getUnsignedBits(i, 20) * 1.0;
    const GTime stamp = GTime::fromTow(tow);
    stamp_ = stamp;

    update_interval_ = UPDATE_INTERVAL[buf.getUnsignedBits(i, 4)];
    sync_ = buf.getUnsignedBits(i, 1);
    refd_ = buf.getUnsignedBits(i, 1);
    iod_ = buf.getUnsignedBits(i, 4);
    provider_id_ = buf.getUnsignedBits(i, 16);
    solution_id_ = buf.getUnsignedBits(i, 4);
    const unsigned int nsats = buf.getUnsignedBits(i, getNumSatBits());

    ROS_DEBUG(
        "orbit_corrections: provider_id: %d, solution_id: %d, sats: %d, tow: %3f",
        provider_id_, solution_id_,
        nsats,
        tow);

    // Decode per satellite
    corrections_.clear();
    for (int j = 0; j < nsats; j++)
    {
      const size_t sat_id = buf.getUnsignedBits(i, getNumSatBits()) + getPrnOffset();
      const unsigned int iode = buf.getUnsignedBits(i, getIodeBits());
      const unsigned int iodcrc = buf.getUnsignedBits(i, getIodcrcBits());
      const double deph0 = buf.getSignedBits(i, 22) * 1e-4;
      const double deph1 = buf.getSignedBits(i, 20) * 4e-4;
      const double deph2 = buf.getSignedBits(i, 20) * 4e-4;
      const double ddeph0 = buf.getSignedBits(i, 21) * 1e-6;
      const double ddeph1 = buf.getSignedBits(i, 19) * 4e-6;
      const double ddeph2 = buf.getSignedBits(i, 19) * 4e-6;

      corrections_[sat_id] = OrbitCorrection(
          sat_id,
          iode,
          iodcrc,
          deph0, deph1, deph2,
          ddeph0, ddeph1, ddeph2);
      ROS_DEBUG(
          " - sat(%ld): iode=%d, iodcrc=%d, "
          "deph=(%0.3f, %0.3f, %0.3f), "
          "ddeph=(%0.3f, %0.3f, %0.3f)",
          sat_id,
          iode,
          iodcrc,
          deph0, deph1, deph2,
          ddeph0, ddeph1, ddeph2);
    }
    return true;
  }
  bool correctOrbit(ECEF &pos, const size_t sat_id, const GTime &time)
  {
    if (corrections_.find(sat_id) == corrections_.end())
      return false;
    const auto t = (stamp_ - time).normalized();
    const double x = corrections_[sat_id].deph0_ + corrections_[sat_id].ddeph0_ * t.toSec();
    const double y = corrections_[sat_id].deph1_ + corrections_[sat_id].ddeph1_ * t.toSec();
    const double z = corrections_[sat_id].deph2_ + corrections_[sat_id].ddeph2_ * t.toSec();
    ROS_DEBUG("- correction [%ld] %0.3f, %0.3f, %0.3f",
              sat_id, x, y, z);
    pos.x() += x;
    pos.y() += y;
    pos.z() += z;

    return true;
  }
};
class RTCM3MessageCorrectionsOrbitGPS : public RTCM3MessageCorrectionsOrbit
{
public:
  int getType() const
  {
    return 1057;
  }

protected:
  int getNumSatBits() const
  {
    return 6;
  }
  int getIodeBits() const
  {
    return 8;
  }
  int getIodcrcBits() const
  {
    return 0;
  }
  int getPrnOffset() const
  {
    return 0;
  }
};

class RTCM3MessageCorrectionsClock : public RTCM3MessageCorrectionsBase
{
protected:
  virtual int getNumSatBits() const = 0;
  virtual int getPrnOffset() const = 0;

  double update_interval_;
  unsigned int sync_;
  unsigned int iod_;
  unsigned int provider_id_;
  unsigned int solution_id_;

  GTime stamp_;

  class ClockCorrection
  {
  public:
    unsigned int sat_id_;
    double dclk0_;
    double dclk1_;
    double dclk2_;

    ClockCorrection()
    {
    }
    ClockCorrection(
        const unsigned int sat_id,
        const double dclk0,
        const double dclk1,
        const double dclk2)
      : sat_id_(sat_id)
      , dclk0_(dclk0)
      , dclk1_(dclk1)
      , dclk2_(dclk2)
    {
    }
  };
  std::map<size_t, ClockCorrection> corrections_;

public:
  using Ptr = std::shared_ptr<RTCM3MessageCorrectionsClock>;
  int getCategory() const
  {
    return Category::CORRECTION_CLOCK;
  }
  bool decode(const Buffer &buf)
  {
    size_t i = 24 + 12;
    double tow = buf.getUnsignedBits(i, 20) * 1.0;
    const GTime stamp = GTime::fromTow(tow);
    stamp_ = stamp;

    update_interval_ = UPDATE_INTERVAL[buf.getUnsignedBits(i, 4)];
    sync_ = buf.getUnsignedBits(i, 1);
    iod_ = buf.getUnsignedBits(i, 4);
    provider_id_ = buf.getUnsignedBits(i, 16);
    solution_id_ = buf.getUnsignedBits(i, 4);
    const unsigned int nsats = buf.getUnsignedBits(i, getNumSatBits());

    ROS_DEBUG(
        "clock_corrections: provider_id: %d, solution_id: %d, sats: %d, tow: %3f",
        provider_id_, solution_id_,
        nsats,
        tow);

    // Decode per satellite
    corrections_.clear();
    for (int j = 0; j < nsats; j++)
    {
      const size_t sat_id = buf.getUnsignedBits(i, getNumSatBits()) + getPrnOffset();
      const double dclk0 = buf.getSignedBits(i, 22) * 1e-4;
      const double dclk1 = buf.getSignedBits(i, 21) * 1e-6;
      const double dclk2 = buf.getSignedBits(i, 27) * 2e-8;

      corrections_[sat_id] = ClockCorrection(
          sat_id, dclk0, dclk1, dclk2);
      ROS_DEBUG(
          " - sat(%ld): "
          "dclk=(%0.6f, %0.6f, %0.6f)",
          sat_id,
          dclk0, dclk1, dclk2);
    }
    return true;
  }
  double getClockCorrections(const size_t sat_id, const GTime &time)
  {
    if (corrections_.find(sat_id) == corrections_.end())
      return 0.0;
    const auto t = (stamp_ - time).normalized();

    return (corrections_[sat_id].dclk0_ +
            corrections_[sat_id].dclk1_ * t.toSec() * t.toSec() +
            corrections_[sat_id].dclk2_ * t.toSec()) /
           CLIGHT;
  }
};
class RTCM3MessageCorrectionsClockGPS : public RTCM3MessageCorrectionsClock
{
public:
  int getType() const
  {
    return 1058;
  }

protected:
  int getNumSatBits() const
  {
    return 6;
  }
  int getPrnOffset() const
  {
    return 0;
  }
};
}  // namespace rtcm3_ros

#endif  // RTCM3_ROS_MESSAGES_CORRECTIONS_H
