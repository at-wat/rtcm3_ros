#ifndef RTCM3_ROS_RTCM3_DECODER_H
#define RTCM3_ROS_RTCM3_DECODER_H

#include <map>
#include <vector>
#include <rtcm3_ros/buffer.h>
#include <rtcm3_ros/rtcm3_messages.h>
#include <rtcm3_ros/ObservationArray.h>

namespace rtcm3_ros
{
class SpawnerBase
{
public:
  using Ptr = std::shared_ptr<SpawnerBase>;
  virtual RTCM3MessageBase::Ptr spawn() const = 0;
};
template <typename T>
class Spawner : public SpawnerBase
{
public:
  RTCM3MessageBase::Ptr spawn() const
  {
    return RTCM3MessageBase::Ptr(new T);
  }
};
class Loader
{
protected:
  std::map<int, SpawnerBase::Ptr> classes_;

public:
  RTCM3MessageBase::Ptr loadClass(const int &type)
  {
    if (classes_.find(type) == classes_.end())
    {
      throw std::runtime_error("Message type not found");
    }
    return classes_[type]->spawn();
  };
  template <typename T>
  void registerClass()
  {
    const int type = T().getType();
    classes_[type] = SpawnerBase::Ptr(new Spawner<T>);
  };
};

class RTCM3Decoder
{
protected:
  Loader decoders_;
  Buffer raw_buffer_;
  std::map<int, rtcm3_ros::RTCM3MessageEphemeridesBase::Ptr> ephemerides_;
  using ObservationCallback = std::function<void(const std::vector<rtcm3_ros::Observation> &)>;
  ObservationCallback cb_observations_;

public:
  RTCM3Decoder()
  {
    decoders_.registerClass<RTCM3MessageEphemeridesGps>();
    decoders_.registerClass<RTCM3MessagePseudoRangeMsm7>();
  }
  void registerObservationsCallback(ObservationCallback cb_observations)
  {
    cb_observations_ = cb_observations;
  }
  void operator<<(const Buffer &input)
  {
    const Buffer raw = raw_buffer_ + input;
    raw_buffer_.clear();

    for (auto it = raw.begin(); it != raw.end(); ++it)
    {
      if (*it != 0xD3)
        continue;
      const size_t remain = raw.end() - it;
      if (remain < 3)
      {
        raw_buffer_ = Buffer(it, raw.end());
        break;
      }
      const size_t length = Buffer(it, it + 3).getUnsignedBits(14, 10);
      if (length + 3 + 3 > remain)
      {
        raw_buffer_ = Buffer(it, raw.end());
        break;
      }
      const Buffer msg(it, it + 3 + length);
      const Buffer crc(it + 3 + length, it + 3 + length + 3);
      if (msg.CRC24Q() != crc.getUnsignedBits(0, 24))
        continue;

      decodeOneMessage(msg);
      it += length;
    }
  }
  void decodeOneMessage(const Buffer &msg)
  {
    const int type = msg.getUnsignedBits(24, 12);
    ROS_DEBUG("RTCM3: message type %d", type);
    try
    {
      auto decoder = decoders_.loadClass(type);
      decoder->decode(msg);
      switch (decoder->getCategory())
      {
        case RTCM3MessageBase::Category::EPHEMERIDES:
        {
          RTCM3MessageEphemeridesBase::Ptr eph = std::dynamic_pointer_cast<RTCM3MessageEphemeridesBase>(decoder);
          ephemerides_[eph->getSatId()] = eph;
          break;
        }
        case RTCM3MessageBase::Category::PSEUDO_RANGE:
        {
          RTCM3MessagePseudoRangeBase::Ptr ranges = std::dynamic_pointer_cast<RTCM3MessagePseudoRangeBase>(decoder);
          ROS_INFO("---");

          std::vector<rtcm3_ros::Observation> observations;
          for (auto &range : *ranges)
          {
            if (ephemerides_.find(range.first) != ephemerides_.end())
            {
              const ECEF sat_pos = ephemerides_[range.first]->getPos(range.second.getTime());
              const double dts = ephemerides_[range.first]->getClockBias(range.second.getTime());
              ROS_INFO("[%2d] r: %10.1f, p: %6.1f, sat: (%11.1f, %11.1f, %11.1f), dts: %0.6f",
                       range.first,
                       range.second.getPseudoRange(),
                       range.second.getPhaseCycle(),
                       sat_pos.x(), sat_pos.y(), sat_pos.z(),
                       dts);

              rtcm3_ros::Observation observation;
              observation.satellite_id = range.first;
              observation.satellite_position.x = sat_pos.x();
              observation.satellite_position.y = sat_pos.y();
              observation.satellite_position.z = sat_pos.z();
              observation.range.pseudo_range = range.second.getPseudoRange();
              observation.range.phase_cycle = range.second.getPhaseCycle();
              observation.range.doppler_frequency = range.second.getDopplerFrequency();
              observation.range.base_frequency = range.second.getBaseFrequency();
              observation.range.wave_length = range.second.getWaveLength();
              observation.range.snr = range.second.getSNR();
              observation.clock_bias = dts;
              observations.push_back(observation);
            }
            else
            {
              ROS_INFO("[%2d] r: %10.1f, p: %6.1f, no ephemerides",
                       range.first,
                       range.second.getPseudoRange(),
                       range.second.getPhaseCycle());
            }
          }
          if (cb_observations_ && observations.size() > 0)
          {
            cb_observations_(observations);
          }
          break;
        }
      }
    }
    catch (std::runtime_error &e)
    {
      ROS_DEBUG("%s", e.what());
    }
  }
};
};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_RTCM3_DECODER_H
