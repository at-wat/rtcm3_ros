#ifndef RTCM3_ROS_RTCM3_MESSAGES_H
#define RTCM3_ROS_RTCM3_MESSAGES_H

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
protected:
  // satellite number
  int sat_;
  // IODE, IODC
  int iode_, iodc_;
  // SV accuracy (URA index)
  int sva_;
  // SV health (0:ok)
  int svh_;
  // GPS/QZS: gps week, GAL: galileo week
  int week_;
  // GPS/QZS: code on L2, GAL/CMP: data sources
  int code_;
  // GPS/QZS: L2 P data flag, CMP: nav type
  int flag_;
  // Toe,Toc,T_trans
  GTime toe_, toc_, ttr_;

  // SV orbit parameters
  double A_, e_, i0_, OMG0_, omg_, M0_, deln_, OMGd_, idot_;
  double crc_, crs_, cuc_, cus_, cic_, cis_;

  // Toe (s) in week
  double toes_;
  // fit interval (h)
  double fit_;
  // SV clock parameters (af0,af1,af2)
  double f0_, f1_, f2_;
  // group delay parameters
  //  GPS/QZS:tgd[0]=TGD
  //  GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1
  //  CMP    :tgd[0]=BGD1,tgd[1]=BGD2
  double tgd_[4];
  // Adot,ndot for CNAV
  double Adot_, ndot_;

public:
  virtual int getType() const = 0;
  virtual bool decode(const Buffer &) = 0;

  ECFC getPos(const int &sat, const GTime &time)
  {
    return ECFC();
  }
};

class RTCM3MessageEphemeridesGps : public RTCM3MessageEphemeridesBase
{
public:
  constexpr int getType() const
  {
    return 1019;
  }
  bool decode(const Buffer &buf)
  {
    size_t i = 24 + 12;
    unsigned int prn = buf.getUnsignedBits(i, 6);
    i += 6;
    const unsigned int week = buf.getUnsignedBits(i, 10);
    i += 10;
    sva_ = buf.getUnsignedBits(i, 4);
    i += 4;
    code_ = buf.getUnsignedBits(i, 2);
    i += 2;
    idot_ = buf.getSignedBits(i, 14) * P2_43 * SC2RAD;
    i += 14;
    iode_ = buf.getUnsignedBits(i, 8);
    i += 8;
    const double toc = buf.getUnsignedBits(i, 16) * 16.0;
    i += 16;
    f2_ = buf.getSignedBits(i, 8) * P2_55;
    i += 8;
    f1_ = buf.getSignedBits(i, 16) * P2_43;
    i += 16;
    f0_ = buf.getSignedBits(i, 22) * P2_31;
    i += 22;
    iodc_ = buf.getUnsignedBits(i, 10);
    i += 10;
    crs_ = buf.getSignedBits(i, 16) * P2_5;
    i += 16;
    deln_ = buf.getSignedBits(i, 16) * P2_43 * SC2RAD;
    i += 16;
    M0_ = buf.getSignedBits(i, 32) * P2_31 * SC2RAD;
    i += 32;
    cuc_ = buf.getSignedBits(i, 16) * P2_29;
    i += 16;
    e_ = buf.getUnsignedBits(i, 32) * P2_33;
    i += 32;
    cus_ = buf.getSignedBits(i, 16) * P2_29;
    i += 16;
    const double sqrtA = buf.getUnsignedBits(i, 32) * P2_19;
    i += 32;
    toes_ = buf.getUnsignedBits(i, 16) * 16.0;
    i += 16;
    cic_ = buf.getSignedBits(i, 16) * P2_29;
    i += 16;
    OMG0_ = buf.getSignedBits(i, 32) * P2_31 * SC2RAD;
    i += 32;
    cis_ = buf.getSignedBits(i, 16) * P2_29;
    i += 16;
    i0_ = buf.getSignedBits(i, 32) * P2_31 * SC2RAD;
    i += 32;
    crc_ = buf.getSignedBits(i, 16) * P2_5;
    i += 16;
    omg_ = buf.getSignedBits(i, 32) * P2_31 * SC2RAD;
    i += 32;
    OMGd_ = buf.getSignedBits(i, 24) * P2_43 * SC2RAD;
    i += 24;
    tgd_[0] = buf.getSignedBits(i, 8) * P2_31;
    i += 8;
    svh_ = buf.getUnsignedBits(i, 6);
    i += 6;
    flag_ = buf.getUnsignedBits(i, 1);
    i += 1;
    fit_ = buf.getUnsignedBits(i, 1) ? 0.0 : 4.0;  // 0:4hr, 1:>4hr

    ROS_WARN("prn: %d", prn);
    return true;
  }
};

};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_RTCM3_MESSAGES_H
