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

#include <sstream>
#include <utility>
#include <algorithm>

#include <diagnostic_updater/publisher.h>

#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

namespace visensor {

/******************************************************************************/
/* Static initialization                                                      */
/******************************************************************************/

std::map<SensorId::SensorId, std::string> VISensorNode::_sensorIdsMap = {
  {SensorId::CAM0, "cam0"},
  {SensorId::CAM1, "cam1"},
  {SensorId::CAM2, "cam2"},
  {SensorId::CAM3, "cam3"},
  {SensorId::IMU0, "imu0"},
  {SensorId::IMU_CAM0, "imu_cam0"},
  {SensorId::IMU_CAM1, "imu_cam1"},
  {SensorId::IMU_CAM2, "imu_cam2"},
  {SensorId::IMU_CAM3, "imu_cam3"},
  {SensorId::CORNER_CAM0, "corner_cam0"},
  {SensorId::CORNER_CAM1, "corner_cam1"},
  {SensorId::CORNER_CAM2, "corner_cam2"},
  {SensorId::CORNER_CAM3, "corner_cam3"},
  {SensorId::DENSE_MATCHER0, "dense_matcher0"},
  {SensorId::EXTERNAL_TRIGGER0, "external_trigger0"},
  {SensorId::SENSOR_STATUS, "sensor_status"},
  {SensorId::SENSOR_CLOCK, "sensor_clock"},
  {SensorId::FLIR0, "flir0"},
  {SensorId::FLIR1, "flir1"},
  {SensorId::FLIR2, "flir2"},
  {SensorId::FLIR3, "flir3"},
  {SensorId::LED_FLASHER0, "lef_flasher0"}
};

/******************************************************************************/
/* Constructors and Destructor                                                */
/******************************************************************************/

  VISensorNode::VISensorNode(const ros::NodeHandle& nh) :
      _nodeHandle(nh) {
    getParameters();
    _imuMinFreq = _imuFrequency - _imuMinFreqPercentage *
      _imuFrequency;
    _imuMaxFreq = _imuFrequency + _imuMaxFreqPercentage *
      _imuFrequency;
    _cameraMinFreq = _cameraFrequency - _cameraMinFreqPercentage *
      _cameraFrequency;
    _cameraMaxFreq = _cameraFrequency + _cameraMaxFreqPercentage *
      _cameraFrequency;
    _driver = std::make_shared<ViSensorDriver>();
    _updater.setHardwareID(_deviceName);
    _updater.force_update();
  }

  VISensorNode::~VISensorNode() {
  }

/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

  void VISensorNode::imuCallback(boost::shared_ptr<ViImuMsg> imuMsg, ViErrorCode
      errorCode) {
    if (!_sensorIdsMap.count(static_cast<SensorId::SensorId>(imuMsg->imu_id))) {
      ROS_WARN_STREAM("VISensorNode::imuCallback(): Unknown sensor ID: "
        << imuMsg->imu_id);
      return;
    }
    if (errorCode == ViErrorCodes::MEASUREMENT_DROPPED) {
      ROS_WARN_STREAM(
        "VISensorNode::imuCallback(): Measurement dropped on sensor: "
        << _sensorIdsMap.at(static_cast<SensorId::SensorId>(imuMsg->imu_id)));
      return;
    }
    if (_imuPublishers.at(static_cast<SensorId::SensorId>(imuMsg->imu_id)).
        getNumSubscribers() > 0) {
      auto imuMsgPtr = boost::make_shared<sensor_msgs::Imu>();
      imuMsgPtr->header.stamp = ros::Time().fromNSec(imuMsg->timestamp);
      imuMsgPtr->header.frame_id =
        _sensorIdsMap.at(static_cast<SensorId::SensorId>(imuMsg->imu_id)) +
        "_link";
      imuMsgPtr->orientation_covariance[0] = -1.0;
      imuMsgPtr->angular_velocity.x = imuMsg->gyro[0];
      imuMsgPtr->angular_velocity.y = imuMsg->gyro[1];
      imuMsgPtr->angular_velocity.z = imuMsg->gyro[2];
      imuMsgPtr->angular_velocity_covariance[0] = 0.0;
      imuMsgPtr->angular_velocity_covariance[1] = 0.0;
      imuMsgPtr->angular_velocity_covariance[2] = 0.0;
      imuMsgPtr->angular_velocity_covariance[3] = 0.0;
      imuMsgPtr->angular_velocity_covariance[4] = 0.0;
      imuMsgPtr->angular_velocity_covariance[5] = 0.0;
      imuMsgPtr->angular_velocity_covariance[6] = 0.0;
      imuMsgPtr->angular_velocity_covariance[7] = 0.0;
      imuMsgPtr->angular_velocity_covariance[8] = 0.0;
      imuMsgPtr->linear_acceleration.x = imuMsg->acc[0];
      imuMsgPtr->linear_acceleration.y = imuMsg->acc[1];
      imuMsgPtr->linear_acceleration.z = imuMsg->acc[2];
      imuMsgPtr->linear_acceleration_covariance[0] = 0.0;
      imuMsgPtr->linear_acceleration_covariance[1] = 0.0;
      imuMsgPtr->linear_acceleration_covariance[2] = 0.0;
      imuMsgPtr->linear_acceleration_covariance[3] = 0.0;
      imuMsgPtr->linear_acceleration_covariance[4] = 0.0;
      imuMsgPtr->linear_acceleration_covariance[5] = 0.0;
      imuMsgPtr->linear_acceleration_covariance[6] = 0.0;
      imuMsgPtr->linear_acceleration_covariance[7] = 0.0;
      imuMsgPtr->linear_acceleration_covariance[8] = 0.0;
      _imuPublishers.at(
        static_cast<SensorId::SensorId>(imuMsg->imu_id)).publish(imuMsgPtr);
      // TODO: Add proper variances in the messages
    }
    _imuFreqs.at(static_cast<SensorId::SensorId>(imuMsg->imu_id))->tick();
    _updater.update();
  }

  void VISensorNode::frameCallback(ViFrame::Ptr frame, ViErrorCode errorCode) {
    if (!_sensorIdsMap.count(static_cast<SensorId::SensorId>(
        frame->camera_id))) {
      ROS_WARN_STREAM("VISensorNode::frameCallback(): Unknown sensor ID: "
        << frame->camera_id);
      return;
    }
    if (errorCode == ViErrorCodes::MEASUREMENT_DROPPED) {
      ROS_WARN_STREAM(
        "VISensorNode::frameCallback(): Measurement dropped on sensor: "
        << _sensorIdsMap.at(static_cast<SensorId::SensorId>(frame->camera_id)));
      return;
    }
    if (_cameraPublishers.at(static_cast<SensorId::SensorId>(frame->camera_id)).
        getNumSubscribers() > 0) {
      auto imgMsg = boost::make_shared<sensor_msgs::Image>();
      if (frame->image_type == MONO8)
        sensor_msgs::fillImage(imgMsg, sensor_msgs::image_encodings::MONO8,
          frame->height, frame->width, frame->width, frame->getImageRawPtr());
      else if (frame->image_type == MONO16) {
        cv::Mat image;
        image.create(frame->height, frame->width, CV_16UC1);
        cv::Mat image_8bit;
        image_8bit.create(frame->height, frame->width, CV_8UC1);
        memcpy(image.data, frame->getImageRawPtr(), frame->width * frame->height * 2);
        sensor_msgs::fillImage(imgMsg, sensor_msgs::image_encodings::MONO16,
          frame->height, frame->width, frame->width * 2, image.data);
      }
      else
        ROS_WARN_STREAM("VISensorNode::frameCallback(): Unknown image type");
      auto ciMsg = boost::make_shared<sensor_msgs::CameraInfo>();
      _cameraPublishers.at(
        static_cast<SensorId::SensorId>(frame->camera_id)).publish(imgMsg,
        ciMsg);
    }
    _cameraFreqs.at(static_cast<SensorId::SensorId>(frame->camera_id))->tick();
    _updater.update();
  }

  void VISensorNode::denseCallback(ViFrame::Ptr frame, ViErrorCode errorCode) {
    if (!_sensorIdsMap.count(static_cast<SensorId::SensorId>(
        frame->camera_id))) {
      ROS_WARN_STREAM("VISensorNode::denseCallback(): Unknown sensor ID: "
        << frame->camera_id);
      return;
    }
    if (errorCode == ViErrorCodes::MEASUREMENT_DROPPED) {
      ROS_WARN_STREAM(
        "VISensorNode::denseCallback(): Measurement dropped on sensor: "
        << _sensorIdsMap.at(static_cast<SensorId::SensorId>(frame->camera_id)));
      return;
    }
  }

  void VISensorNode::frameCornerCallback(ViFrame::Ptr frame, ViCorner::Ptr
      corners) {
  }

  void VISensorNode::triggerCallback(ViExternalTriggerMsg::Ptr trigger) {
  }

  bool VISensorNode::init() {
    _driver->getAutoDiscoveryDeviceList(_hostnames);
    if (_hostnames.empty()) {
      ROS_WARN_STREAM("VISensorNode::init(): No device found");
      return false;
    }
    _driver->init(_hostnames.front());
    _sensorIds = _driver->getListOfSensorIDs();
    _cameraIds = _driver->getListOfCameraIDs();
    _denseIds = _driver->getListOfDenseIDs();
    _imuIds = _driver->getListOfImuIDs();
    _triggerIds = _driver->getListOfTriggerIDs();
    _fpgaId = _driver->getFpgaId();
    ROS_INFO_STREAM("VISensorNode::init(): Found a sensor with address: "
      << _hostnames.front());
    std::stringstream sensorIdsStream;
    for (auto it = _sensorIds.cbegin(); it != _sensorIds.cend(); ++it) {
      if (_sensorIdsMap.count(*it))
        sensorIdsStream << _sensorIdsMap.at(*it) << " ";
      else {
        ROS_WARN_STREAM("VISensorNode::init(): Unknown sensor ID: " << *it);
        return false;
      }
    }
    ROS_INFO_STREAM("VISensorNode::init(): Connected sensors: "
      << sensorIdsStream.str());
    _driver->setCameraCallback(boost::bind(&VISensorNode::frameCallback, this,
      _1, _2));
    _driver->setDenseMatcherCallback(boost::bind(&VISensorNode::denseCallback,
      this, _1, _2));
    _driver->setImuCallback(boost::bind(&VISensorNode::imuCallback, this,
      _1, _2));
    _driver->setFramesCornersCallback(
      boost::bind(&VISensorNode::frameCornerCallback, this, _1, _2));
    _driver->setExternalTriggerCallback(
      boost::bind(&VISensorNode::triggerCallback, this, _1));
    try {
      _driver->setCameraCalibrationSlot(0);
    }
    catch (const visensor::exceptions::SensorException& e) {
      ROS_WARN_STREAM("VISensorNode::init(): " << e.what());
    }
    for (auto it = _imuIds.cbegin(); it != _imuIds.cend(); ++it) {
      _imuPublishers.insert(std::make_pair(*it,
        _nodeHandle.advertise<sensor_msgs::Imu>(_sensorIdsMap.at(*it),
        _queueDepth)));
      _imuFreqs.insert(std::make_pair(*it,
        std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
        _sensorIdsMap.at(*it), _updater,
        diagnostic_updater::FrequencyStatusParam(&_imuMinFreq, &_imuMaxFreq,
        0.1, 10))));
    }
    for (auto it = _cameraIds.cbegin(); it != _cameraIds.cend(); ++it) {
      _cameraImageTransports.insert(std::make_pair(*it,
        image_transport::ImageTransport(_nodeHandle)));
      _cameraPublishers.insert(std::make_pair(*it,
        _cameraImageTransports.at(*it).advertiseCamera(_sensorIdsMap.at(*it) +
        "/image_raw", _queueDepth)));
      _cameraFreqs.insert(std::make_pair(*it,
        std::make_shared<diagnostic_updater::HeaderlessTopicDiagnostic>(
        _sensorIdsMap.at(*it), _updater,
        diagnostic_updater::FrequencyStatusParam(&_cameraMinFreq,
        &_cameraMaxFreq, 0.1, 10))));
    }
    _driver->startAllImus(_imuFrequency);
    _driver->startAllCameras(_cameraFrequency);
    return true;
  }

  void VISensorNode::cleanup() {
    _hostnames.clear();
    _sensorIds.clear();
    _cameraIds.clear();
    _denseIds.clear();
    _imuIds.clear();
    _triggerIds.clear();
    _fpgaId = 0;
  }

  void VISensorNode::spin() {
    init();
    ros::spin();
  }

  void VISensorNode::getParameters() {
    _nodeHandle.param<std::string>("ros/frame_id", _frameId, "/visensor_link");
    _nodeHandle.param<int>("ros/queue_depth", _queueDepth, 100);
    _nodeHandle.param<double>("connection/retry_timeout", _retryTimeout, 1);
    _nodeHandle.param<double>("diagnostics/imu_min_freq_percentage",
      _imuMinFreqPercentage, 0.2);
    _nodeHandle.param<double>("diagnostics/imu_max_freq_percentage",
      _imuMaxFreqPercentage, 0.2);
    _nodeHandle.param<double>("diagnostics/camera_min_freq_percentage",
      _cameraMinFreqPercentage, 0.2);
    _nodeHandle.param<double>("diagnostics/camera_max_freq_percentage",
      _cameraMaxFreqPercentage, 0.2);
    _nodeHandle.param<std::string>("sensor/device_name", _deviceName,
      "VI-sensor");
    _nodeHandle.param<int>("sensor/imu_frequency", _imuFrequency,
      IMU_FREQUENCY);
    _nodeHandle.param<int>("sensor/camera_frequency", _cameraFrequency,
      CAMERA_FREQUENCY);
  }

}
