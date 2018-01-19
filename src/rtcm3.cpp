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
#include <rtcm3/BinaryStream.h>

#include <boost/asio.hpp>

class RTCM3Node
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  ros::Publisher pub_stream_;
  ros::Timer interval_;

  std::string frame_id_;
  std::string ip_;
  int port_;

  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::deadline_timer timer_;

  boost::asio::streambuf buf_;

  void onTimeoutConnect(const boost::system::error_code &error)
  {
    if (error)
      return;
    ROS_ERROR("Connection failed: %s", error.message().c_str());
    ros::shutdown();
  }
  void onTimeout(const boost::system::error_code &error)
  {
    if (error)
      return;
    ROS_ERROR("Connection timedout: %s", error.message().c_str());
    ros::shutdown();
  }
  void onConnect(const boost::system::error_code &error)
  {
    timer_.cancel();
    ROS_INFO("Connected");
    receivePacket();
  }
  void receivePacket()
  {
    timer_.cancel();

    timer_.expires_from_now(boost::posix_time::seconds(120.0));
    timer_.async_wait(
        boost::bind(&RTCM3Node::onTimeout, this,
                    boost::asio::placeholders::error));
    boost::asio::async_read(
        socket_,
        buf_,
        boost::asio::transfer_at_least(1),
        boost::bind(
            &RTCM3Node::onRead,
            this,
            boost::asio::placeholders::error));
  };
  void onRead(const boost::system::error_code &error)
  {
    timer_.cancel();
    if (error == boost::asio::error::eof)
    {
      ROS_ERROR("Connection closed");
      ros::shutdown();
      return;
    }
    else if (error)
    {
      ROS_ERROR("Connection errored");
      ros::shutdown();
      return;
    }
    auto length = buf_.size();

    rtcm3::BinaryStream array;
    const uint8_t *p = boost::asio::buffer_cast<const uint8_t *>(buf_.data());
    array.header.frame_id = frame_id_;
    array.header.stamp = ros::Time::now();
    array.data = std::vector<uint8_t>(&p[0], &p[length]);

    pub_stream_.publish(array);

    buf_.consume(length);
    receivePacket();
  }
  void cbTimer(const ros::TimerEvent &event)
  {
    boost::system::error_code ec;
    io_service_.poll(ec);
    if (ec)
    {
      ROS_ERROR("IO error");
      ros::shutdown();
    }
  }

public:
  RTCM3Node()
    : pnh_("~")
    , nh_("")
    , socket_(io_service_)
    , timer_(io_service_)
  {
    pnh_.param("frame_id", frame_id_, std::string("ant"));
    pnh_.param("ip", ip_, std::string("127.0.0.1"));
    pnh_.param("port", port_, 8020);

    pub_stream_ = nh_.advertise<rtcm3::BinaryStream>("rtcm3", 10);

    ROS_INFO("Connecting to %s:%d", ip_.c_str(), port_);

    boost::asio::ip::tcp::endpoint endpoint(
        boost::asio::ip::address::from_string(ip_), port_);
    timer_.expires_from_now(boost::posix_time::seconds(10.0));
    timer_.async_wait(
        boost::bind(&RTCM3Node::onTimeoutConnect, this,
                    boost::asio::placeholders::error));
    socket_.async_connect(
        endpoint,
        boost::bind(&RTCM3Node::onConnect,
                    this,
                    boost::asio::placeholders::error));

    interval_ = nh_.createTimer(ros::Duration(0.1), &RTCM3Node::cbTimer, this);
  };
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtcm3");
  RTCM3Node node;

  ros::spin();

  return 1;
}
