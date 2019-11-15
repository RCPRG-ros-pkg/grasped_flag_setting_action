cmake_minimum_required(VERSION 2.8.3)
project(rtt_grasped_action_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm grasped_action_msgs)

ros_generate_rtt_typekit(grasped_action_msgs)
ros_generate_rtt_service_proxies(grasped_action_msgs)

orocos_generate_package(
  DEPENDS grasped_action_msgs
  DEPENDS_TARGETS rtt_roscomm
)
