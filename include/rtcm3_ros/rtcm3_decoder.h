#ifndef RTCM3_ROS_RTCM3_DECODER_H
#define RTCM3_ROS_RTCM3_DECODER_H

#include <map>
#include <memory>
#include <vector>

#include <rtcm3_ros/buffer.h>
#include <rtcm3_ros/rtcm3_messages.h>

#include <rtcm3_ros/IonosphericDelay.h>
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
  std::vector<Buffer> raw_buffer_;
  std::map<int, RTCM3MessageEphemeridesBase::Ptr> ephemerides_;
  RTCM3MessageCorrectionsOrbit::Ptr corrections_orbit_;
  RTCM3MessageCorrectionsClock::Ptr corrections_clock_;
  IonoDelay iono_;

  using ObservationCallback = std::function<void(const std::vector<rtcm3_ros::Observation> &)>;
  ObservationCallback cb_observations_;
  using IonoCallback = std::function<void(const std::vector<rtcm3_ros::IonosphericDelayGridPoint> &)>;
  IonoCallback cb_iono_;

  bool require_ssr_;

public:
  RTCM3Decoder(const bool require_ssr = true)
    : require_ssr_(require_ssr)
  {
    decoders_.registerClass<RTCM3MessageEphemeridesGps>();
    decoders_.registerClass<RTCM3MessagePseudoRangeMsm7>();
    decoders_.registerClass<RTCM3MessageCorrectionsOrbitGPS>();
    decoders_.registerClass<RTCM3MessageCorrectionsClockGPS>();
    decoders_.registerClass<RTCM3MessageSbas>();
  }
  void registerObservationsCallback(ObservationCallback cb_observations)
  {
    cb_observations_ = cb_observations;
  }
  void registerIonoCallback(IonoCallback cb_iono)
  {
    cb_iono_ = cb_iono;
  }
  void process(const size_t id, const Buffer &input)
  {
    if (raw_buffer_.size() <= id)
      raw_buffer_.resize(id + 1);

    const Buffer raw = raw_buffer_[id] + input;
    raw_buffer_[id].clear();

    for (auto it = raw.begin(); it != raw.end(); ++it)
    {
      if (*it != 0xD3)
        continue;
      const size_t remain = raw.end() - it;
      if (remain < 3)
      {
        raw_buffer_[id] = Buffer(it, raw.end());
        break;
      }
      const size_t length = Buffer(it, it + 3).getUnsignedBitsConst(14, 10);
      if (length + 3 + 3 > remain)
      {
        raw_buffer_[id] = Buffer(it, raw.end());
        break;
      }
      const Buffer msg(it, it + 3 + length);
      const Buffer crc(it + 3 + length, it + 3 + length + 3);
      if (msg.CRC24Q() != crc.getUnsignedBitsConst(0, 24))
      {
        ROS_DEBUG("CRC missmatch (size: %lu)", length);
        continue;
      }

      decodeOneMessage(msg);
      it += length;
    }
  }
  void decodeOneMessage(const Buffer &msg)
  {
    const int type = msg.getUnsignedBitsConst(24, 12);
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
        case RTCM3MessageBase::Category::CORRECTION_ORBIT:
        {
          corrections_orbit_ = std::dynamic_pointer_cast<RTCM3MessageCorrectionsOrbit>(decoder);
          break;
        }
        case RTCM3MessageBase::Category::CORRECTION_CLOCK:
        {
          corrections_clock_ = std::dynamic_pointer_cast<RTCM3MessageCorrectionsClock>(decoder);
          break;
        }
        case RTCM3MessageBase::Category::PSEUDO_RANGE:
        {
          RTCM3MessagePseudoRangeBase::Ptr ranges = std::dynamic_pointer_cast<RTCM3MessagePseudoRangeBase>(decoder);
          ROS_DEBUG("---");
          if ((!corrections_orbit_ || !corrections_clock_) && require_ssr_)
          {
            ROS_DEBUG("skipping until receiving ssr (orbit: %d, clock: %d)",
                      corrections_orbit_ ? 1 : 0, corrections_clock_ ? 1 : 0);
            break;
          }

          std::vector<rtcm3_ros::Observation> observations;
          for (auto &range : *ranges)
          {
            if (ephemerides_.find(range.first) != ephemerides_.end())
            {
              const Duration dt(0.001);
              ECEF sat_pos =
                  ephemerides_[range.first]->getPos(range.second.getTime());
              ECEF sat_pos2 =
                  ephemerides_[range.first]->getPos(range.second.getTime() + dt);
              if (!corrections_orbit_->correctOrbit(sat_pos, range.first, range.second.getTime()))
                continue;
              if (!corrections_orbit_->correctOrbit(sat_pos2, range.first, range.second.getTime() + dt))
                continue;

              const double dts =
                  ephemerides_[range.first]->getClockBias(range.second.getTime()) +
                  corrections_clock_->getClockCorrections(range.first, range.second.getTime());
              ROS_DEBUG("[%2d] r: %10.1f, p: %6.1f, sat: (%11.1f, %11.1f, %11.1f), dts: %0.6f",
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
              observation.satellite_velocity.x = (sat_pos2.x() - sat_pos.x()) / dt.toSec();
              observation.satellite_velocity.y = (sat_pos2.y() - sat_pos.y()) / dt.toSec();
              observation.satellite_velocity.z = (sat_pos2.z() - sat_pos.z()) / dt.toSec();
              observation.range.pseudo_range = range.second.getPseudoRange();
              observation.range.phase_cycle = range.second.getPhaseCycle();
              observation.range.doppler_frequency = range.second.getDopplerFrequency();
              observation.range.base_frequency = range.second.getBaseFrequency();
              observation.range.wave_length = range.second.getWaveLength();
              observation.range.snr = range.second.getSNR();
              observation.clock_bias = dts;
              observation.frequency = range.second.getBaseFrequency();
              observations.push_back(observation);
            }
            else
            {
              ROS_DEBUG("[%2d] r: %10.1f, p: %6.1f, no ephemerides",
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
        case RTCM3MessageBase::Category::SBAS:
        {
          RTCM3MessageSbas::Ptr sbas = std::dynamic_pointer_cast<RTCM3MessageSbas>(decoder);
          sbas->persistent(iono_);
          std::vector<rtcm3_ros::IonosphericDelayGridPoint> igps;
          for (const auto &d : iono_)
          {
            rtcm3_ros::IonosphericDelayGridPoint igp;
            igp.latitude = d.first.first;
            igp.longitude = d.first.second;
            igp.delay = d.second;
            igps.push_back(igp);
          }
          if (cb_iono_ && igps.size() > 0)
          {
            cb_iono_(igps);
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
