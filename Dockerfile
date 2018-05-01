FROM ros:kinetic

COPY package.xml /catkin_ws/src/rtcm3_ros/package.xml
RUN apt-get -qq update \
  && rosdep install --from-paths /catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY ./ /catkin_ws/src/rtcm3_ros
RUN bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash \
  && cd /catkin_ws \
  && catkin_make_isolated --install-space /opt/ros/${ROS_DISTRO} --install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf devel_isolated build_isolated"
