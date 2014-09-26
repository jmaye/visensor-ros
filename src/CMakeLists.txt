remake_ros_package(
  visensor
  DEPENDS roscpp rospy rosbash std_msgs sensor_msgs diagnostic_updater
  EXTRA_BUILD_DEPENDS libvisensor-dev
  EXTRA_RUN_DEPENDS libvisensor
  DESCRIPTION "Driver for the VI-sensor."
)
remake_ros_package_config_generate(PACKAGE visensor EXTRA_CFLAGS -std=c++0x)
