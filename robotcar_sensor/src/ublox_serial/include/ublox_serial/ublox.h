#ifndef _UBLOX_H_
#define _UBLOX_H_

#include <string>

typedef struct RMC
{
    std::string str;
    std::string header;
    std::string gpsTime;
    std::string status;
    std::string lat;
    std::string nsIndicator;
    std::string lon;
    std::string ewIndicator;
    std::string spd;
    std::string cog;
    std::string date;
    std::string mv;
    std::string mvEW;
    std::string poseMode;
    std::string navStatus;
    std::string checksum;
    std::string CrLf;
}  RMC;






#endif // !_UBLOX_H_





