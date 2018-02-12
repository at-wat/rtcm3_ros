#ifndef RTCM3_ROS_CONSTANTS_H
#define RTCM3_ROS_CONSTANTS_H

namespace rtcm3_ros
{
const double PI = 3.1415926535897932;   // pi
const double D2R = (PI / 180.0);        // deg to rad
const double R2D = (180.0 / PI);        // rad to deg
const double CLIGHT = 299792458.0;      // speed of light (m/s)
const double SC2RAD = 3.1415926535898;  // semi-circle to radian (IS-GPS)
const double AU = 149597870691.0;       // 1 AU (m)
const double AS2R = (D2R / 3600.0);     // arc sec to radian

const double OMGE = 7.2921151467E-5;  // earth angular velocity (IS-GPS) (rad/s)
const double OMGE_GLO = 7.292115E-5;
const double OMGE_GAL = 7.2921151467E-5;

const double RE_WGS84 = 6378137.0;              // earth semimajor axis (WGS84) (m)
const double FE_WGS84 = (1.0 / 298.257223563);  // earth flattening (WGS84)

const double HION = 350000.0;  // ionosphere height (m)

const double PRUNIT_GPS = 299792.458;      // rtcm ver.3 unit of gps pseudorange (m)
const double PRUNIT_GLO = 599584.916;      // rtcm ver.3 unit of glonass pseudorange (m)
const double RANGE_MS = (CLIGHT * 0.001);  // range in 1 ms

const double RE_GLO = 6378136.0;       // radius of earth (m)
const double MU_GPS = 3.9860050E14;    // gravitational constant
const double MU_GLO = 3.9860044E14;    // gravitational constant
const double MU_GAL = 3.986004418E14;  // earth gravitational constant
const double MU_CMP = 3.986004418E14;  // earth gravitational constant
const double J2_GLO = 1.0826257E-3;    // 2nd zonal harmonic of geopot

};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_CONSTANTS_H
