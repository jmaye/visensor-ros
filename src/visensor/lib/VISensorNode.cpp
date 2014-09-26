/******************************************************************************
 * Copyright (C) 2014 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

#include "VISensorNode.h"

#include <diagnostic_updater/publisher.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <sensor_msgs/Imu.h>

#include <visensor/visensor.hpp>

namespace visensor {

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  VISensorNode::VISensorNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh) {
    getParameters();
    _driver = std::make_shared<ViSensorDriver>();
    _updater.setHardwareID(_deviceName);
    _dpFreq.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
      "data_packet", _updater,
      diagnostic_updater::FrequencyStatusParam(&_dpMinFreq, &_dpMaxFreq,
      0.1, 10)));
    _updater.force_update();
  }

  VISensorNode::~VISensorNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void VISensorNode::init() {
    try {
      _driver->init();
    }
    catch (const visensor::exceptions& e) {
      ROS_ERROR_STREAM(e.what());
      exit(1);
    }
//    list_of_available_sensors_ = drv_.getListOfSensorIDs();
//    list_of_camera_ids_ = drv_.getListOfCameraIDs();
//    list_of_dense_ids_ = drv_.getListOfDenseIDs();
//    list_of_imu_ids_ = drv_.getListOfImuIDs();
//    list_of_trigger_ids_ = drv_.getListOfTriggerIDs();

//    std::string rootdir = ros::package::getPath("visensor_node");
//    std::string tempCameraInfoFileName;

//    pub_time_host_ = nh_.advertise<visensor_node::visensor_time_host>("time_host", -1);

//    try {
//      drv_.setCameraCallback(boost::bind(&ViSensor::frameCallback, this, _1, _2));
//      drv_.setDenseMatcherCallback(boost::bind(&ViSensor::denseCallback, this, _1, _2));
//      drv_.setImuCallback(boost::bind(&ViSensor::imuCallback, this, _1, _2));
//      drv_.setFramesCornersCallback(boost::bind(&ViSensor::frameCornerCallback, this, _1, _2));
//      drv_.setExternalTriggerCallback(boost::bind(&ViSensor::triggerCallback, this, _1));
//      drv_.setCameraCalibrationSlot(0); // 0 is factory calibration
//    } catch (visensor::exceptions const &ex) {
//      ROS_WARN("%s", ex.what());
//    }

//    // initialize cameras
//    for (auto camera_id : list_of_camera_ids_) {
//      ros::NodeHandle nhc_temp(nh_, ROS_CAMERA_NAMES.at(camera_id));
//      nhc_.insert(std::pair<SensorId::SensorId, ros::NodeHandle>(camera_id, nhc_temp));
//      image_transport::ImageTransport itc_temp(nhc_[camera_id]);
//      itc_.insert(std::pair<SensorId::SensorId, image_transport::ImageTransport>(camera_id, itc_temp));
//      ROS_INFO("Register publisher for camera %u with topic name %s", camera_id,
//               ROS_CAMERA_NAMES.at(camera_id).c_str());
//      image_transport::CameraPublisher image_pub = (itc_.at(camera_id)).advertiseCamera("image_raw", 100);
//      image_pub_.insert(std::pair<SensorId::SensorId, image_transport::CameraPublisher>(camera_id, image_pub));

//      precacheViCalibration(camera_id);

//      sensor_msgs::CameraInfo cinfo_temp;
//      if (getRosCameraConfig(camera_id, cinfo_temp))
//        ROS_INFO_STREAM("Read calibration for "<< ROS_CAMERA_NAMES.at(camera_id));
//      else
//        ROS_INFO_STREAM("Could not read calibration for "<< ROS_CAMERA_NAMES.at(camera_id));

//      cinfo_.insert(std::pair<SensorId::SensorId, sensor_msgs::CameraInfo>(camera_id, cinfo_temp));

//      ros::Publisher temp_pub;
//      temp_pub = nhc_temp.advertise<visensor_node::visensor_calibration>("calibration", -1);
//      calibration_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(camera_id, temp_pub));
//    }

//    //Generate Stereo ROS config, assuming than cam0 and cam1 are in fronto-parallel stereo configuration
//    if (getRosStereoCameraConfig(SensorId::CAM0, cinfo_.at(SensorId::CAM0),
//                                 SensorId::CAM1, cinfo_.at(SensorId::CAM1)))
//      ROS_INFO("Generated ROS Stereo Calibration, assuming cam0 and cam1 are a stereo pair.");
//    else
//      ROS_INFO("Could not read stereo calibration for cam0 and cam1.");

//    // initialize dense cameras
//    for (auto dense_id : list_of_dense_ids_) {
//      ros::NodeHandle nhc_temp(nh_, "dense");
//      nhc_.insert(std::pair<SensorId::SensorId, ros::NodeHandle>(dense_id, nhc_temp));
//      image_transport::ImageTransport itc_temp(nhc_[dense_id]);
//      itc_.insert(std::pair<SensorId::SensorId, image_transport::ImageTransport>(dense_id, itc_temp));
//      ROS_INFO_STREAM("Register publisher for dense " << dense_id);
//      image_transport::CameraPublisher image_pub = (itc_.at(dense_id)).advertiseCamera("image_raw", 100);
//      image_pub_.insert(std::pair<SensorId::SensorId, image_transport::CameraPublisher>(dense_id, image_pub));

//      //sensor_msgs::CameraInfo cinfo_temp;
//      //if (getRosCameraConfig(dense_id, cinfo_temp))
//      //  ROS_INFO_STREAM("Read calibration for "<< dense_id);
//      //else
//      //  ROS_INFO_STREAM("Could not read calibration for "<< dense_id);

//      //TODO(omaris) Adapt camera info messages accordingly
//      //cinfo_.insert(std::pair<SensorId::SensorId, sensor_msgs::CameraInfo>(dense_id, cinfo_temp));
//    }

//    // Initialize imus
//    for (auto imu_id : list_of_imu_ids_) {
//      ros::Publisher temp_pub;
//      temp_pub = nh_.advertise<sensor_msgs::Imu>(ROS_IMU_NAMES.at(imu_id), -1);
//      printf("register publisher for imu %u\n", imu_id);
//      imu_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(imu_id, temp_pub));
//      temp_pub = nh_.advertise<visensor_node::visensor_imu>("cust_" + ROS_IMU_NAMES.at(imu_id), -1);
//      imu_custom_pub_.insert(std::pair<visensor::SensorId::SensorId, ros::Publisher>(imu_id, temp_pub));
//    }

//    trigger_pub_ = nh_.advertise<visensor_node::visensor_trigger>("external_trigger", -1);
//    calibration_service_ = nh_.advertiseService("get_camera_calibration", &ViSensor::calibrationServiceCallback, this);

//    // init dynamic reconfigure
//    dr_srv_.setCallback(boost::bind(&ViSensor::configCallback, this, _1, _2));
  }

  void VISensorNode::spin() {
    init();
    ros::spin();
  }

  void VISensorNode::getParameters() {
    _nodeHandle.param<std::string>("ros/frame_id", _frameId, "/visensor_link");
    _nodeHandle.param<int>("ros/queue_depth", _queueDepth, 100);
    _nodeHandle.param<double>("connection/retry_timeout", _retryTimeout, 1);
    _nodeHandle.param<double>("diagnostics/dp_min_freq", _dpMinFreq, 2777.73);
    _nodeHandle.param<double>("diagnostics/dp_max_freq", _dpMaxFreq, 4166.6);
    _nodeHandle.param<std::string>("sensor/device_name", _deviceName,
      "VI-sensor");
  }

}
