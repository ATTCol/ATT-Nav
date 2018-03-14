/*
 * LMS1xx.cpp
 *
 *  Created on: 09-08-2010
 *  Author: Konrad Banachowicz
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include <csignal>
#include <cstdio>
#include <LMS1xx/LMS1xx.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
// #include "laser_geometry/laser_geometry.h"
//#include <limits>
//#include <string>

#define DEG2RAD M_PI/180.0

int main(int argc, char **argv)
{
  // laser data
  LMS1xx laser1;
  LMS1xx laser2;
  scanCfg cfg1;
  scanCfg cfg2;
  scanOutputRange outputRange1;
  scanOutputRange outputRange2;
  scanDataCfg dataCfg1;
  scanDataCfg dataCfg2;
  sensor_msgs::LaserScan scan1_msg;
  sensor_msgs::LaserScan scan2_msg;

  // parameters
  std::string host1;
  std::string host2;
  std::string frame_id1;
  std::string frame_id2;
  int port;

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::Publisher scan_pub1 = nh.advertise<sensor_msgs::LaserScan>("scan1", 1);
  ros::Publisher scan_pub2 = nh.advertise<sensor_msgs::LaserScan>("scan2", 1);

  n.param<std::string>("host1", host1, "192.168.0.3");
  n.param<std::string>("host2", host2, "192.168.0.4");
  n.param<std::string>("frame_id1", frame_id1, "scanner_left");
  n.param<std::string>("frame_id2", frame_id2, "scanner_right");
  n.param<int>("port", port, 2111);


  while (ros::ok())
  {
    //laser1 connect
    ROS_INFO_STREAM("Connecting to laser1 at " << host1);
    laser1.connect(host1, port);
    if (!laser1.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    //laser2 connect
    ROS_INFO_STREAM("Connecting to laser2 at " << host2);
    laser2.connect(host2, port);
    if (!laser2.isConnected())
    {
      ROS_WARN("Unable to connect, retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    //laser1 login
    ROS_DEBUG("Logging in to laser1.");
    laser1.login();
    cfg1 = laser1.getScanCfg();
    outputRange1 = laser1.getScanOutputRange();

    if (cfg1.scaningFrequency != 5000)
    {
      laser1.disconnect();
      ROS_WARN("Unable to get laser1 output range. Retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    //laser2 login
    ROS_DEBUG("Logging in to laser2.");
    laser2.login();
    cfg2 = laser2.getScanCfg();
    outputRange2 = laser2.getScanOutputRange();

    if (cfg2.scaningFrequency != 5000)
    {
      laser2.disconnect();
      ROS_WARN("Unable to get laser2 output range. Retrying.");
      ros::Duration(1).sleep();
      continue;
    }

    //laser1 scan_msg
    ROS_INFO("Connected to laser1.");

    ROS_DEBUG("Laser1 configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg1.scaningFrequency, cfg1.angleResolution, cfg1.startAngle, cfg1.stopAngle);
    ROS_DEBUG("Laser1 output range:angleResolution %d, startAngle %d, stopAngle %d",
              outputRange1.angleResolution, outputRange1.startAngle, outputRange1.stopAngle);

    scan1_msg.header.frame_id = frame_id1;
    scan1_msg.range_min = 0.01;
    scan1_msg.range_max = 20.0;
    scan1_msg.scan_time = 100.0 / cfg1.scaningFrequency;
    scan1_msg.angle_increment = (double)outputRange1.angleResolution / 10000.0 * DEG2RAD;
    scan1_msg.angle_min = (double)outputRange1.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
    scan1_msg.angle_max = (double)outputRange1.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;

    ROS_DEBUG_STREAM("Device resolution is " << (double)outputRange1.angleResolution / 10000.0 << " degrees.");
    ROS_DEBUG_STREAM("Device frequency is " << (double)cfg1.scaningFrequency / 100.0 << " Hz");

    //laser2 scan_msg
    ROS_INFO("Connected to laser2.");

    ROS_DEBUG("Laser2 configuration: scaningFrequency %d, angleResolution %d, startAngle %d, stopAngle %d",
              cfg2.scaningFrequency, cfg2.angleResolution, cfg2.startAngle, cfg2.stopAngle);
    ROS_DEBUG("Laser2 output range:angleResolution %d, startAngle %d, stopAngle %d",
              outputRange2.angleResolution, outputRange2.startAngle, outputRange2.stopAngle);

    scan2_msg.header.frame_id = frame_id2;
    scan2_msg.range_min = 0.01;
    scan2_msg.range_max = 20.0;
    scan2_msg.scan_time = 100.0 / cfg2.scaningFrequency;
    scan2_msg.angle_increment = (double)outputRange2.angleResolution / 10000.0 * DEG2RAD;
    scan2_msg.angle_min = (double)outputRange2.startAngle / 10000.0 * DEG2RAD - M_PI / 2;
    scan2_msg.angle_max = (double)outputRange2.stopAngle / 10000.0 * DEG2RAD - M_PI / 2;

    ROS_DEBUG_STREAM("Device resolution is " << (double)outputRange2.angleResolution / 10000.0 << " degrees.");
    ROS_DEBUG_STREAM("Device frequency is " << (double)cfg2.scaningFrequency / 100.0 << " Hz");

    //laser1 angleRange
    int angle_range1 = outputRange1.stopAngle - outputRange1.startAngle;
    int num_values1 = angle_range1 / outputRange1.angleResolution ;

    if (angle_range1 % outputRange1.angleResolution == 0)
    {
      // Include endpoint
      ++num_values1;
    }

    //laser2 angleRange
    int angle_range2 = outputRange2.stopAngle - outputRange2.startAngle;
    int num_values2 = angle_range2 / outputRange2.angleResolution ;

    if (angle_range2 % outputRange2.angleResolution == 0)
    {
      // Include endpoint
      ++num_values2;
    }

    //laser1 resize
    scan1_msg.ranges.resize(num_values1);
    scan1_msg.intensities.resize(num_values1);

    scan1_msg.time_increment =
      (outputRange1.angleResolution / 10000.0)
      / 360.0
      / (cfg1.scaningFrequency / 100.0);

    ROS_DEBUG_STREAM("Time increment is " << static_cast<int>(scan1_msg.time_increment * 1000000) << " microseconds");

    //laser2 resize
    scan2_msg.ranges.resize(num_values2);
    scan2_msg.intensities.resize(num_values2);

    scan2_msg.time_increment =
      (outputRange2.angleResolution / 10000.0)
      / 360.0
      / (cfg2.scaningFrequency / 100.0);

    ROS_DEBUG_STREAM("Time increment is " << static_cast<int>(scan2_msg.time_increment * 1000000) << " microseconds");

    //laser1 datacfg
    dataCfg1.outputChannel = 1;
    dataCfg1.remission = true;
    dataCfg1.resolution = 1;
    dataCfg1.encoder = 0;
    dataCfg1.position = true;
    dataCfg1.deviceName = false;
    dataCfg1.outputInterval = 1;

    ROS_DEBUG("Setting scan data configuration.");
    laser1.setScanDataCfg(dataCfg1);

    ROS_DEBUG("Starting measurements.");
    laser1.startMeas();

    ROS_DEBUG("Waiting for ready status.");
    ros::Time ready_status_timeout = ros::Time::now() + ros::Duration(5);

    //laser2 datacfg
    dataCfg2.outputChannel = 1;
    dataCfg2.remission = true;
    dataCfg2.resolution = 1;
    dataCfg2.encoder = 0;
    dataCfg2.position = true;
    dataCfg2.deviceName = false;
    dataCfg2.outputInterval = 1;

    ROS_DEBUG("Setting scan data configuration.");
    laser2.setScanDataCfg(dataCfg2);

    ROS_DEBUG("Starting measurements.");
    laser2.startMeas();

    //laser1 status
    status_t stat1 = laser1.queryStatus();
    ros::Duration(1.0).sleep();
    if (stat1 != ready_for_measurement)
    {
      ROS_WARN("Laser not ready. Retrying initialization.");
      laser1.disconnect();
      ros::Duration(1).sleep();
      continue;
    }

    //laser2 status
    status_t stat2 = laser2.queryStatus();
    ros::Duration(1.0).sleep();
    if (stat2 != ready_for_measurement)
    {
      ROS_WARN("Laser not ready. Retrying initialization.");
      laser2.disconnect();
      ros::Duration(1).sleep();
      continue;
    }

    //laser1 start scanner
    ROS_DEBUG("Starting device.");
    laser1.startDevice(); // Log out to properly re-enable system after config

    ROS_DEBUG("Commanding continuous measurements.");
    laser1.scanContinous(1);

    //laser2 start scannner
    ROS_DEBUG("Starting device.");
    laser2.startDevice(); // Log out to properly re-enable system after config

    ROS_DEBUG("Commanding continuous measurements.");
    laser2.scanContinous(1);
//rospy.Time.no

    while (ros::ok())
    {
      ros::Time start = ros::Time::now();

      //laser1 data-aquisitie
      scan1_msg.header.stamp = start;
      ++scan1_msg.header.seq;

      scanData dataScanner1;
      ROS_DEBUG("Reading scan data from scanner 1.");
      if (laser1.getScanData(&dataScanner1))
      {
        for (int i = 0; i < dataScanner1.dist_len1; i++)
        {//rospy.Time.no
          scan1_msg.ranges[i] = dataScanner1.dist1[i] * 0.001;
        }

        for (int i = 0; i < dataScanner1.rssi_len1; i++)
        {
          scan1_msg.intensities[i] = dataScanner1.rssi1[i];
        }

        ROS_DEBUG("Publishing scan data.");
        scan_pub1.publish(scan1_msg);
      }
      else
      {
        ROS_ERROR("Laser1 timed out on delivering scan, attempting to reinitialize.");
        break;
      }

      ros::spinOnce();

      //laser2 data-aquisitie
      scan2_msg.header.stamp = start;
      ++scan2_msg.header.seq;

      scanData dataScanner2;
      ROS_DEBUG("Reading scan data.");
      if (laser2.getScanData(&dataScanner2))
      {
        for (int i = 0; i < dataScanner2.dist_len1; i++)
        {
          scan2_msg.ranges[i] = dataScanner2.dist1[i] * 0.001;
        }

        for (int i = 0; i < dataScanner2.rssi_len1; i++)
        {
          scan2_msg.intensities[i] = dataScanner2.rssi1[i];
        }

        ROS_DEBUG("Publishing scan data.");
        scan_pub2.publish(scan2_msg);
      }
      else
      {
        ROS_ERROR("Laser2 timed out on delivering scan, attempting to reinitialize.");
        break;
      }

      ros::spinOnce();
    }

    laser1.scanContinous(0);
    laser1.stopMeas();
    laser1.disconnect();
    laser2.scanContinous(0);
    laser2.stopMeas();
    laser2.disconnect();
  }

  return 0;

}
