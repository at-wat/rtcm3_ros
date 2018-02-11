#ifndef RTCM3_ROS_MESSAGES_PSEUDO_RANGES_H
#define RTCM3_ROS_MESSAGES_PSEUDO_RANGES_H

#include <map>
#include <utility>
#include <vector>
#include <rtcm3_ros/messages/rtcm3_messages_base.h>

namespace rtcm3_ros
{
class RTCM3MessagePseudoRangeMsm : public RTCM3MessagePseudoRangeBase
{
protected:
  // station id
  int station_;
  // issue of data station
  uint8_t iod_;
  // cumulative session transmitting time
  uint8_t time_s_;
  // clock steering indicator
  uint8_t clk_str_;
  // external clock indicator
  uint8_t clk_ext_;
  // divergence free smoothing indicator
  uint8_t smooth_;
  // soothing interval
  uint8_t tint_s_;
  // satellites
  std::vector<uint8_t> sats_;
  // signals
  std::vector<uint8_t> sigs_;
  // cell mask
  std::map<std::pair<uint8_t, uint8_t>, uint8_t> cellmask_;

  // sync flag
  uint8_t sync_;

public:
  virtual int getType() const = 0;
  virtual bool decode(const Buffer &buf) = 0;
  int decodeHeader(const Buffer &buf)
  {
    size_t i = 24 + 12;

    station_ = buf.getUnsignedBits(i, 12);
    i += 12;

    // Sat system dependent part
    i += 30;

    // Common part
    sync_ = buf.getUnsignedBits(i, 1);
    i += 1;
    iod_ = buf.getUnsignedBits(i, 3);
    i += 3;
    time_s_ = buf.getUnsignedBits(i, 7);
    i += 7;
    clk_str_ = buf.getUnsignedBits(i, 2);
    i += 2;
    clk_ext_ = buf.getUnsignedBits(i, 2);
    i += 2;
    smooth_ = buf.getUnsignedBits(i, 1);
    i += 1;
    tint_s_ = buf.getUnsignedBits(i, 3);
    i += 3;
    for (uint8_t j = 0; j < 64; j++)
    {
      const int mask = buf.getUnsignedBits(i, 1);
      i += 1;
      if (mask)
        sats_.push_back(j);
    }
    for (uint8_t j = 0; j < 32; j++)
    {
      const int mask = buf.getUnsignedBits(i, 1);
      i += 1;
      if (mask)
        sigs_.push_back(j);
    }
    for (auto &sat : sats_)
    {
      for (auto &sig : sigs_)
      {
        cellmask_[std::pair<uint8_t, uint8_t>(sat, sig)] = buf.getUnsignedBits(i, 1);
        i += 1;
      }
    }

    return i;
  }
  virtual Container::iterator begin() = 0;
  virtual Container::iterator end() = 0;
};

class RTCM3MessagePseudoRangeMsm7 : public RTCM3MessagePseudoRangeMsm
{
protected:
  GTime stamp_;
  Container ranges_;

public:
  constexpr int getType() const
  {
    return 1077;
  }
  bool decode(const Buffer &buf)
  {
    const char *SIGNAL_TYPE_NAMES[] =
        {
          "", "1C", "1P", "1W", "1Y", "1M", "", "2C", "2P", "2W", "2Y", "2M",
          "", "", "2S", "2L", "2X", "", "", "", "", "5I", "5Q", "5X",
          "", "", "", "", "", "1S", "1L", "1X"
        };
    const int SIGNAL_TYPE_BANDS[] =
        {
          0, 1, 1, 1, 1, 1, 1, 1, 1, 1,
          1, 1, 1, 1, 2, 2, 2, 2, 2, 2,
          2, 2, 2, 2, 3, 3, 3, 5, 5, 5,
          4, 4, 4, 4, 4, 4, 4, 6, 6, 6,
          2, 2, 4, 4, 3, 3, 3, 1, 1, 0
        };
    const double GPS_BAND_FREQUENCIES[] =
        {
          1.57542e9,  // L1/E1
          1.22760e9,  // L2
          1.17645e9,  // L5/E5a
          1.27875e9,  // E6/LEX
          1.20714e9,  // E5b
          1.191795e9  // E5a+b
        };

    const double tow = buf.getUnsignedBits(24 + 12 + 12, 30) * 0.001;
    const GTime stamp = GTime::fromTow(tow);

    size_t i = decodeHeader(buf);
    ROS_DEBUG("paseudo_ranges: sats: %ld, sigs: %ld, tow: %0.3lf", sats_.size(), sigs_.size(), stamp.getTow());

    std::map<size_t, double> pseudo_range_base;
    for (auto &sat : sats_)
    {
      pseudo_range_base[sat] = buf.getUnsignedBits(i, 8) * RANGE_MS;
      i += 8;
    }
    // Decode extended info
    std::map<int, unsigned int> ex;
    for (auto &sat : sats_)
    {
      ex[sat] = buf.getUnsignedBits(i, 4);
      i += 4;
    }
    for (auto &sat : sats_)
    {
      pseudo_range_base[sat] += buf.getUnsignedBits(i, 10) * P2_10 * RANGE_MS;
      i += 10;
    }
    std::map<size_t, double> phase_range_rate_base;
    for (auto &sat : sats_)
    {
      phase_range_rate_base[sat] += buf.getSignedBits(i, 14) * 1.0;
      i += 14;
    }

    // Decode per signal data
    int i_pseudo_range = i;
    int i_phase_range = i_pseudo_range + cellmask_.size() * 20;
    int i_lock_time = i_phase_range + cellmask_.size() * 24;
    int i_half = i_lock_time + cellmask_.size() * 10;
    int i_cnr = i_half + cellmask_.size() * 1;
    int i_phase_range_rate = i_cnr + cellmask_.size() * 10;
    for (auto &satsig : cellmask_)
    {
      const auto pseudo_range_raw = buf.getSignedBits(i_pseudo_range, 20);
      const double pseudo_range = pseudo_range_raw * P2_29 * RANGE_MS +
                                  pseudo_range_base[satsig.first.first];
      i_pseudo_range += 20;
      const auto phase_range_raw = buf.getSignedBits(i_phase_range, 24);
      const double phase_range = phase_range_raw * P2_31 * RANGE_MS;
      i_phase_range += 24;
      const auto lock_time = buf.getUnsignedBits(i_lock_time, 10);
      i_lock_time += 10;
      const auto half = buf.getUnsignedBits(i_half, 1);
      i_half += 1;
      const auto snr_raw = buf.getSignedBits(i_cnr, 10);
      const double snr = snr_raw * 0.0625 * 4.0;
      i_cnr += 10;
      const auto phase_range_rate_raw = buf.getSignedBits(i_phase_range_rate, 15);
      const double phase_range_rate = -phase_range_rate_raw * 0.0001 +
                                      phase_range_rate_base[satsig.first.first];
      i_phase_range_rate += 15;

      const double base_frequency =
          GPS_BAND_FREQUENCIES[SIGNAL_TYPE_BANDS[satsig.first.first]];
      const double wave_length = CLIGHT / base_frequency;
      const double doppler_shift = phase_range_rate / wave_length;

      ROS_DEBUG(" - sat(%d), sig(%d): pseudo_range=%0.3lf, phase_range=%0.3lf, doppler=%0.1lf",
                satsig.first.first, satsig.first.second,
                pseudo_range, phase_range, doppler_shift);

      ranges_[satsig.first.first] = PseudoRange(stamp, snr, 0, 0, pseudo_range, phase_range, doppler_shift);
    }

    return true;
  }
  Container::iterator begin()
  {
    return ranges_.begin();
  }
  Container::iterator end()
  {
    return ranges_.end();
  }
};

};  // namespace rtcm3_ros

#endif  // RTCM3_ROS_MESSAGES_PSEUDO_RANGES_H
