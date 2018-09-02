#ifndef RTCM3_ROS_MESSAGES_SBAS_H
#define RTCM3_ROS_MESSAGES_SBAS_H

#include <ros/ros.h>
#include <rtcm3_ros/messages/rtcm3_messages_base.h>

#include <sstream>

namespace
{
typedef struct
{
  int week, tow;
  int prn;
  unsigned char msg[29];
} SbasMsg;
}

// 4001
namespace rtcm3_ros
{
class RTCM3MessageSbas : public RTCM3MessageBase
{
public:
  using Ptr = std::shared_ptr<RTCM3MessageSbas>;
  uint8_t sbas_type_;

  IonoMask mask_;
  std::map<size_t, float> delays_;
  uint32_t iodi_;
  uint32_t band_;

  int getCategory() const
  {
    return Category::SBAS;
  }
  int getType() const
  {
    return 4001;
  }
  void persistent(IonoDelay &iono)
  {
    switch (sbas_type_)
    {
      case 18:
      {
        iono.updateMask(band_, mask_);
        break;
      }
      case 26:
      {
        for (const auto &d : delays_)
          iono.updateDelay(band_, d.first, iodi_, d.second);
        break;
      }
    }
  }
  bool decode(const Buffer &buf_raw)
  {
    SbasMsg msg;
    memcpy(&msg, buf_raw.data() + 6, sizeof(SbasMsg));

    std::vector<uint8_t> sbas_msg(&msg.msg[0], &msg.msg[29]);
    Buffer buf(sbas_msg.begin(), sbas_msg.end());

    size_t pos = 8;
    sbas_type_ = buf.getUnsignedBits(pos, 6);
    ROS_DEBUG("SBAS %u, type: %u, size: %u", msg.prn, sbas_type_, buf.size());
    switch (sbas_type_)
    {
      case 18:
      {
        pos = 18;
        band_ = buf.getUnsignedBits(pos, 4);
        iodi_ = buf.getUnsignedBits(pos, 2);
        IonoMask mask(band_, iodi_);
        std::stringstream ss;
        ss << "iono grid mask:" << std::endl;
        ss << "  band: " << band_ << std::endl;
        ss << "  iodi: " << iodi_ << std::endl;
        ss << "  mask: ";
        for (uint32_t i = 0; i < 201; ++i)
        {
          uint32_t bit = buf.getUnsignedBits(pos, 1);
          if (bit)
            mask.addMask(i);
          ss << bit;
          if (i % 32 == 31)
            ss << std::endl
               << "        ";
        }
        mask_ = mask;
        ROS_DEBUG("%s", ss.str().c_str());
        break;
      }
      case 26:
      {
        pos = 14;
        band_ = buf.getUnsignedBits(pos, 4);
        const uint32_t block = buf.getUnsignedBits(pos, 4);
        iodi_ = buf.getUnsignedBitsConst(217, 2);
        std::stringstream ss;
        ss << "iono delay:" << std::endl;
        ss << "  band: " << band_ << std::endl;
        ss << "  block: " << block << std::endl;
        ss << "  iodi: " << iodi_ << std::endl;
        ss << "  delay: [";
        for (int i = 0; i < 15; ++i)
        {
          const size_t index = block * 15 + i;
          const uint32_t delay_raw = buf.getUnsignedBits(pos, 9);
          const uint32_t give = buf.getUnsignedBits(pos, 4);
          const float delay = delay_raw == 0x1ff ? 0.0 : delay_raw * 0.125;
          ss << delay << " ";
          delays_[index] = delay;
        }
        ss << "]";
        ROS_DEBUG("%s", ss.str().c_str());
        break;
      }
    }
    return true;
  }

protected:
};
}  // namespace rtcm3_ros

#endif  // RTCM3_ROS_MESSAGES_SBAS_H
