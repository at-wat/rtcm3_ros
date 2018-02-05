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

#include <string>

unsigned int getUnsignedBits(
    const rtcm3_ros::BinaryStream::ConstPtr &msg, size_t pos, size_t len)
{
  unsigned int bits = 0;
  for (size_t i = pos; i < pos + len; i++)
  {
    bits = (bits << 1) + ((msg->data[i / 8] >> (7 - i % 8)) & 1u);
  }
  return bits;
}
int getSignedBits(
    const rtcm3_ros::BinaryStream::ConstPtr &msg, size_t pos, size_t len)
{
  unsigned int bits = getUnsignedBits(msg, pos, len);
  if (!(bits & (1u << (len - 1))))
  {
    return (int)bits;
  }
  return (int)(bits | (~0u << len));
}

class RTCM3Decode
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_stream_;
  std::map<std::string, ros::Publisher> pub_;

  void cbStream(const rtcm3_ros::BinaryStream::ConstPtr &msg)
  {
    size_t i = 0;
    while (true)
    {
      for (; i < msg->data.size(); ++i)
      {
        if (msg->data[i] == 0xD3)
          break;
      }
      if (i == msg->data.size())
        return;
      const size_t length = getUnsignedBits(msg, i * 8 + 14, 10);
      if (length == 0 || length + 3 + 3 > msg->data.size() - i)
      {
        ++i;
        continue;
      }
      const int type = getUnsignedBits(msg, i * 8 + 24, 12);
      ROS_INFO("RTCM3: message type %d, length %d /%d",
               type,
               static_cast<int>(length) + 3 + 3,
               static_cast<int>(msg->data.size() - i));
      i += 3;

      i += length + 3;
    }
  }

public:
  RTCM3Decode()
  {
    sub_stream_ = nh_.subscribe("rtcm3", 10, &RTCM3Decode::cbStream, this);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtcm3_decode");
  RTCM3Decode node;

  ros::spin();

  return 1;
}
