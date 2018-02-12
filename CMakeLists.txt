cmake_minimum_required(VERSION 2.8.3)
project(rtcm3_ros)

find_package(
  catkin
  REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    message_generation
)
find_package(Boost 1.53 REQUIRED system)

add_message_files(
  FILES
    BinaryStream.msg
    PseudoRange.msg
    Observation.msg
    ObservationArray.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  DEPENDS
    roscpp
    std_msgs
    geometry_msgs
)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include)

add_executable(rtcm3 src/rtcm3.cpp)
target_link_libraries(rtcm3 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(rtcm3 rtcm3_ros_generate_messages)

add_executable(rtcm3_decode src/rtcm3_decode.cpp)
target_link_libraries(rtcm3_decode ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(rtcm3_decode rtcm3_ros_generate_messages)

install(TARGETS
    rtcm3
    rtcm3_decode
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
