/*
 * Copyright (c) 2017, Meiji University, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <rtcm3_ros/BinaryStream.h>
#include <rtcm3_ros/ObservationArray.h>
#include <rtcm3_ros/IonosphericDelay.h>

#include <rtcm3_ros/rtcm3_decoder.h>

#include <string>

class RTCM3Decode
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> sub_stream_;
  ros::Publisher pub_observations_;
  ros::Publisher pub_iono_;

  rtcm3_ros::RTCM3Decoder dec_;

  void cbStream(const rtcm3_ros::BinaryStream::ConstPtr &msg, const size_t stream_id)
  {
    dec_.process(stream_id, rtcm3_ros::Buffer(msg->data));
  }
  void cbObservations(const std::vector<rtcm3_ros::Observation> &observations)
  {
    rtcm3_ros::ObservationArray array;
    array.header.stamp = ros::Time::now();
    array.header.frame_id = "antenna";
    array.observations = observations;

    pub_observations_.publish(array);
  }
  void cbIono(const std::vector<rtcm3_ros::IonosphericDelayGridPoint> &igps)
  {
    rtcm3_ros::IonosphericDelay iono;
    iono.header.stamp = ros::Time::now();
    iono.header.frame_id = "earth";
    iono.igps = igps;

    pub_iono_.publish(iono);
  }

public:
  RTCM3Decode()
    : nh_("")
    , pnh_("~")
  {
    int num_input;
    pnh_.param("num_input", num_input, 1);
    if (num_input == 1)
    {
      sub_stream_.push_back(
          nh_.subscribe<rtcm3_ros::BinaryStream>(
              "rtcm3", 100,
              boost::bind(&RTCM3Decode::cbStream, this, _1, 0)));
    }
    else
    {
      for (int i = 0; i < num_input; ++i)
        sub_stream_.push_back(
            nh_.subscribe<rtcm3_ros::BinaryStream>(
                "rtcm3_" + std::to_string(i), 100,
                boost::bind(&RTCM3Decode::cbStream, this, _1, i)));
    }
    pub_observations_ = nh_.advertise<rtcm3_ros::ObservationArray>("observations", 100);
    dec_.registerObservationsCallback(boost::bind(&RTCM3Decode::cbObservations, this, _1));

    pub_iono_ = nh_.advertise<rtcm3_ros::IonosphericDelay>("iono", 100);
    dec_.registerIonoCallback(boost::bind(&RTCM3Decode::cbIono, this, _1));
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtcm3_decode");
  RTCM3Decode node;

  ros::spin();

  return 1;
}
