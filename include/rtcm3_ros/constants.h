#ifndef RTCM3_ROS_CONSTATNTS_H
#define RTCM3_ROS_CONSTATNTS_H

namespace rtcm3_ros
{
const double P2_5 = 0.03125;                 // 2^-5
const double P2_6 = 0.015625;                // 2^-6
const double P2_10 = 0.0009765625;           // 2^-10
const double P2_11 = 4.882812500000000e-04;  // 2^-11
const double P2_15 = 3.051757812500000e-05;  // 2^-15
const double P2_17 = 7.629394531250000e-06;  // 2^-17
const double P2_19 = 1.907348632812500e-06;  // 2^-19
const double P2_20 = 9.536743164062500e-07;  // 2^-20
const double P2_21 = 4.768371582031250e-07;  // 2^-21
const double P2_23 = 1.192092895507810e-07;  // 2^-23
const double P2_24 = 5.960464477539063e-08;  // 2^-24
const double P2_27 = 7.450580596923828e-09;  // 2^-27
const double P2_29 = 1.862645149230957e-09;  // 2^-29
const double P2_30 = 9.313225746154785e-10;  // 2^-30
const double P2_31 = 4.656612873077393e-10;  // 2^-31
const double P2_32 = 2.328306436538696e-10;  // 2^-32
const double P2_33 = 1.164153218269348e-10;  // 2^-33
const double P2_34 = 5.820766091346740E-11;  // 2^-34
const double P2_35 = 2.910383045673370e-11;  // 2^-35
const double P2_38 = 3.637978807091710e-12;  // 2^-38
const double P2_39 = 1.818989403545856e-12;  // 2^-39
const double P2_40 = 9.094947017729280e-13;  // 2^-40
const double P2_43 = 1.136868377216160e-13;  // 2^-43
const double P2_46 = 1.421085471520200E-14;  // 2^-46
const double P2_48 = 3.552713678800501e-15;  // 2^-48
const double P2_50 = 8.881784197001252e-16;  // 2^-50
const double P2_55 = 2.775557561562891e-17;  // 2^-55
const double P2_59 = 1.734723475976810E-18;  // 2^-59

const double PI = 3.1415926535897932;   // pi
const double D2R = (PI / 180.0);        // deg to rad
const double R2D = (180.0 / PI);        // rad to deg
const double CLIGHT = 299792458.0;      // speed of light (m/s)
const double SC2RAD = 3.1415926535898;  // semi-circle to radian (IS-GPS)
const double AU = 149597870691.0;       // 1 AU (m)
const double AS2R = (D2R / 3600.0);     // arc sec to radian

const double OMGE = 7.2921151467E-5;  // earth angular velocity (IS-GPS) (rad/s)

const double RE_WGS84 = 6378137.0;              // earth semimajor axis (WGS84) (m)
const double FE_WGS84 = (1.0 / 298.257223563);  // earth flattening (WGS84)

const double HION = 350000.0;  // ionosphere height (m)

const double PRUNIT_GPS = 299792.458;      // rtcm ver.3 unit of gps pseudorange (m)
const double PRUNIT_GLO = 599584.916;      // rtcm ver.3 unit of glonass pseudorange (m)
const double RANGE_MS = (CLIGHT * 0.001);  // range in 1 ms
};                                         // namespace rtcm3_ros

#endif  // RTCM3_ROS_CONSTATNTS_H
