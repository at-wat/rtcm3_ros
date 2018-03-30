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

#include <boost/asio.hpp>

class RTCM3ServerNode
{
private:
  ros::NodeHandle pnh_;
  ros::NodeHandle nh_;
  ros::Subscriber sub_stream_;
  ros::Timer interval_;

  int port_;

  boost::asio::io_service io_service_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_;
  std::shared_ptr<boost::asio::ip::tcp::socket> socket_next_;
  std::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_;

  std::list<std::shared_ptr<boost::asio::ip::tcp::socket>> sockets_;

  void cbStream(const rtcm3_ros::BinaryStream::ConstPtr &msg)
  {
    if (sockets_.size() == 0)
      return;
    std::map<std::shared_ptr<boost::asio::ip::tcp::socket>, bool> to_be_removed;
    for (auto &s : sockets_)
    {
      try
      {
        boost::asio::write(
            *s,
            boost::asio::buffer(boost::asio::const_buffer(msg->data.data(), msg->data.size())));
        to_be_removed[s] = false;
      }
      catch (std::runtime_error &e)
      {
        ROS_ERROR("Write error: %s", e.what());
        to_be_removed[s] = true;
      }
    }
    sockets_.erase(
        std::remove_if(
            sockets_.begin(), sockets_.end(),
            [&to_be_removed](std::shared_ptr<boost::asio::ip::tcp::socket> &s) -> bool
            {
              return to_be_removed[s];
            }),
        sockets_.end());
  }
  void onAccept(
      const boost::system::error_code &error)
  {
    if (error)
    {
      ROS_ERROR("Accept error");
      ros::shutdown();
      return;
    }
    ROS_INFO("New rtcm3 client connected");
    sockets_.push_back(socket_next_);
    accept();
  }
  void accept()
  {
    socket_next_ = std::shared_ptr<boost::asio::ip::tcp::socket>(
        new boost::asio::ip::tcp::socket(io_service_));

    acceptor_->async_accept(
        *socket_next_,
        boost::bind(&RTCM3ServerNode::onAccept, this, boost::asio::placeholders::error));
  }

public:
  RTCM3ServerNode()
    : pnh_("~")
    , nh_("")
  {
    sub_stream_ = nh_.subscribe("rtcm3", 10, &RTCM3ServerNode::cbStream, this);

    pnh_.param("port", port_, 8020);

    boost::asio::ip::tcp::endpoint endpoint(
        boost::asio::ip::address::from_string("127.0.0.1"), port_);

    ROS_INFO("Listening %d", port_);
    acceptor_.reset(new boost::asio::ip::tcp::acceptor(io_service_, endpoint));
    accept();

    interval_ = nh_.createTimer(ros::Duration(0.1), &RTCM3ServerNode::cbTimer, this);
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtcm3_server");
  RTCM3ServerNode node;

  ros::spin();

  return 1;
}
