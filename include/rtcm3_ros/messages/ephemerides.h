#ifndef RTCM3_ROS_MESSAGES_EPHEMERIDES_H
#define RTCM3_ROS_MESSAGES_EPHEMERIDES_H

#include <rtcm3_ros/messages/rtcm3_messages_base.h>

namespace rtcm3_ros
{
class RTCM3MessageEphemeridesGps : public RTCM3MessageEphemeridesBase
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
  GTime week_;
  // GPS/QZS: code on L2, GAL/CMP: data sources
  int code_;
  // GPS/QZS: L2 P data flag, CMP: nav type
  int flag_;
  // Toe,Toc,T_trans
  GTime toe_, toc_, ttr_;

  // SV orbit parameters
  double A_, e_, i0_, OMG0_, omg_, M0_, deln_, OMGd_, idot_;
  double crc_, crs_, cuc_, cus_, cic_, cis_;

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
    const uint64_t toc = buf.getUnsignedBits(i, 16) * 16;
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
    const uint64_t toes = buf.getUnsignedBits(i, 16) * 16;
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

    week_ = GTime::from10bitsWeek(week);
    toe_ = GTime::fromTow(toes * 1000000000000, week_);
    toc_ = GTime::fromTow(toc * 1000000000000, week_);
    A_ = sqrtA * sqrtA;

    const auto pos = getPos(toe_);
    ROS_WARN(" - [prn: %d] %0.3lf, %0.3lf, %0.3lf", prn, pos.x(), pos.y(), pos.z());
    return true;
  }
  ECEF getPos(const GTime &time) const
  {
    double M, E, Ek, sinE, cosE, u, r, i, O, sin2u, cos2u, x, y, sinO, cosO, cosi, mu, omge;
    double xg, yg, zg, sino, coso;
    int n, sys, prn;
    const Duration tk = time - toc_;

    mu = MU_GPS;
    omge = OMGE;

    M = M0_ + (sqrt(mu / (A_ * A_ * A_)) + deln_) * tk.toSec();
    for (n = 0, E = M, Ek = 0.0; fabs(E - Ek) > 1e-14 && n < 30; n++)
    {
      Ek = E;
      E -= (E - e_ * sin(E) - M) / (1.0 - e_ * cos(E));
    }
    sinE = sin(E);
    cosE = cos(E);

    u = atan2(sqrt(1.0 - e_ * e_) * sinE, cosE - e_) + omg_;
    r = A_ * (1.0 - e_ * cosE);
    i = i0_ + idot_ * tk.toSec();
    sin2u = sin(2.0 * u);
    cos2u = cos(2.0 * u);
    u += cus_ * sin2u + cuc_ * cos2u;
    r += crs_ * sin2u + crc_ * cos2u;
    i += cis_ * sin2u + cic_ * cos2u;
    x = r * cos(u);
    y = r * sin(u);
    cosi = cos(i);

    O = OMG0_ + (OMGd_ - omge) * tk.toSec() - omge * toe_.getTow();
    sinO = sin(O);
    cosO = cos(O);
    ECEF pos(
        x * cosO - y * cosi * sinO,
        x * sinO + y * cosi * cosO,
        y * sin(i));

    const double dts =
        f0_ + f1_ * tk.toSec() + f2_ * tk.toSec() * tk.toSec() -
        2.0 * sqrt(mu * A_) * e_ * sinE / pow(CLIGHT, 2.0);
    return pos;
  }
  double getVariance()
  {
    const double URA_VALUE[] = {
      2.4, 3.4, 4.85, 6.85,
      9.65, 13.65, 24.0, 48.0,
      96.0, 192.0, 384.0, 768.0,
      1536.0, 3072.0, 6144.0
    };
    if (0 > sva_ || sva_ < 15)
      return pow(6144.8, 2.0);
    return pow(URA_VALUE[sva_], 2.0);
  }
};

};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_MESSAGES_EPHEMERIDES_H
