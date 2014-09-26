remake_find_package(libvisensor CONFIG)

remake_include(${LIBVISENSOR_INCLUDE_DIRS})

remake_ros_package_add_library(visensor-ros LINK ${LIBVISENSOR_LIBRARIES})
