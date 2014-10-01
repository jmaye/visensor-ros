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

/** \file VISensorNode.h
    \brief This file defines the VISensorNode class which implements the
           VI-sensor node.
  */

#ifndef VISENSOR_NODE_H
#define VISENSOR_NODE_H

#include <string>
#include <memory>
#include <vector>
#include <cstdint>
#include <map>

#include <ros/ros.h>

#include <diagnostic_updater/diagnostic_updater.h>

#include <image_transport/image_transport.h>

#include <boost/shared_ptr.hpp>

#include <visensor/visensor.hpp>

namespace diagnostic_updater {
  class HeaderlessTopicDiagnostic;
}

namespace visensor {

  /** The class VISensorNode implements the Vi-sensor node.
      \brief VI-sensor node
    */
  class VISensorNode {
  public:
    /** \name Constructors/destructor
      @{
      */
    /// Constructor
    VISensorNode(const ros::NodeHandle& nh);
    /// Copy constructor
    VISensorNode(const VISensorNode& other) = delete;
    /// Copy assignment operator
    VISensorNode& operator = (const VISensorNode& other) = delete;
    /// Move constructor
    VISensorNode(VISensorNode&& other) = delete;
    /// Move assignment operator
    VISensorNode& operator = (VISensorNode&& other) = delete;
    /// Destructor
    virtual ~VISensorNode();
    /** @}
      */

    /** \name Methods
      @{
      */
    /// Spin once
    void spin();
    /** @}
      */

  protected:
    /** \name Protected methods
      @{
      */
    /// Retrieves parameters
    void getParameters();
    /// Init the VI-sensor
    bool init();
    /// Clear stored sensor information
    void cleanup();
    /// IMU callback
    void imuCallback(boost::shared_ptr<ViImuMsg> imuMsg, ViErrorCode errorCode);
    /// Frame callback
    void frameCallback(ViFrame::Ptr frame, ViErrorCode errorCode);
    /// Dense callback
    void denseCallback(ViFrame::Ptr frame, ViErrorCode errorCode);
    /// Frame corner callback
    void frameCornerCallback(ViFrame::Ptr frame, ViCorner::Ptr corners);
    /// Trigger callback
    void triggerCallback(ViExternalTriggerMsg::Ptr trigger);
    /** @}
      */

    /** \name Protected members
      @{
      */
    /// ROS node handle
    ros::NodeHandle _nodeHandle;
    /// Frame ID
    std::string _frameId;
    /// Device name
    std::string _deviceName;
    /// Retry timeout for connection failure
    double _retryTimeout;
    /// Diagnostic updater
    diagnostic_updater::Updater _updater;
    /// Queue depth
    int _queueDepth;
    /// Handle on the VI-sensor low-level driver
    std::shared_ptr<ViSensorDriver> _driver;
    /// Connected VI-sensors IPs
    ViDeviceList _hostnames;
    /// Sensor Ids
    std::vector<SensorId::SensorId> _sensorIds;
    /// Camera Ids
    std::vector<SensorId::SensorId> _cameraIds;
    /// Dense Ids
    std::vector<SensorId::SensorId> _denseIds;
    /// IMU Ids
    std::vector<SensorId::SensorId> _imuIds;
    /// Trigger Ids
    std::vector<SensorId::SensorId> _triggerIds;
    /// FPGA Id
    uint32_t _fpgaId;
    /// SensorId map to string
    static std::map<SensorId::SensorId, std::string> _sensorIdsMap;
    /// IMU publishers
    std::map<SensorId::SensorId, ros::Publisher> _imuPublishers;
    /// Frequency diagnostic for IMUs
    std::map<SensorId::SensorId,
      std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> >_imuFreqs;
    /// IMU packet minimum frequency percentage
    double _imuMinFreqPercentage;
    /// IMU packet minimum frequency
    double _imuMinFreq;
    /// IMU packet maximum frequency percentage
    double _imuMaxFreqPercentage;
    /// IMU packet maximum frequency
    double _imuMaxFreq;
    /// IMU frequency
    int _imuFrequency;
    /// Camera image transports
    std::map<SensorId::SensorId, image_transport::ImageTransport>
      _cameraImageTransports;
    /// Camera publishers
    std::map<SensorId::SensorId, image_transport::CameraPublisher>
      _cameraPublishers;
    /// Frequency diagnostic for cameras
    std::map<SensorId::SensorId,
      std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> >
      _cameraFreqs;
    /// Camera packet minimum frequency percentage
    double _cameraMinFreqPercentage;
    /// Camera packet minimum frequency
    double _cameraMinFreq;
    /// Camera packet maximum frequency percentage
    double _cameraMaxFreqPercentage;
    /// Camera packet maximum frequency
    double _cameraMaxFreq;
    /// Camera frequency
    int _cameraFrequency;
    /** @}
      */

  };

}

#endif // VISENSOR_NODE_H
