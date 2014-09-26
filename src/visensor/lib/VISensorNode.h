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

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace diagnostic_updater {
  class HeaderlessTopicDiagnostic;
}

namespace visensor {

  class ViSensorDriver;

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
    void init();
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
    /// Frequency diagnostic for data packet
    std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> _dpFreq;
    /// Data packet minimum frequency
    double _dpMinFreq;
    /// Data packet maximum frequency
    double _dpMaxFreq;
    /// Queue depth
    int _queueDepth;
    /// Data packet counter
    long _dataPacketCounter;
    /// Handle on the VI-sensor low-level driver
    std::shared_ptr<ViSensorDriver> _driver;
    /** @}
      */

  };

}

#endif // VISENSOR_NODE_H
