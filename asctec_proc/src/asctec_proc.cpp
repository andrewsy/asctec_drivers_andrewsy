/*
 *  AscTec Autopilot Processor
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "asctec_proc/asctec_proc.h"

namespace asctec
{

AsctecProc::AsctecProc(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  ROS_INFO("Starting AsctecProc"); 

  ros::NodeHandle nh_rawdata  (nh_, rawdata_namespace_);
  ros::NodeHandle nh_procdata (nh_, procdata_namespace_);

  // **** initialize vaiables

  engaged_ = false;
  prev_state_ = OFF;

  ctrl_input_msg_ = boost::make_shared<asctec_msgs::CtrlInput>();

  ctrl_input_msg_->roll  = 0;
  ctrl_input_msg_->pitch = 0;
  ctrl_input_msg_->yaw   = 0;
  ctrl_input_msg_->ctrl  = int(0b1100); // FIXME This is from CtrlInput.msg - use a param
  
  // **** register subscribers

  imuCalcDataSubscriber_ = nh_rawdata.subscribe(imuCalcDataTopic_, 10, &AsctecProc::imuCalcDataCallback, this);
  ll_status_subscriber_  = nh_rawdata.subscribe(ll_status_topic_,   1, &AsctecProc::llStatusCallback,    this);

  cmd_thrust_subscriber_ = nh_procdata.subscribe(cmd_thrust_topic_, 1, &AsctecProc::cmdThrustCallback, this);
  cmd_yaw_subscriber_    = nh_procdata.subscribe(cmd_yaw_topic_,    1, &AsctecProc::cmdYawCallback,    this);
  state_subscriber_      = nh_procdata.subscribe(state_topic_,      1, &AsctecProc::stateCallback,     this);

  // *** register publishers

  imuPublisher_            = nh_procdata.advertise<sensor_msgs::Imu>(imuTopic_, 10);
  heightPublisher_         = nh_procdata.advertise<asctec_msgs::Height>(heightTopic_, 10);
  heightFilteredPublisher_ = nh_procdata.advertise<asctec_msgs::Height>(heightFilteredTopic_, 10);

  ctrl_input_publisher_ = nh_rawdata.advertise<asctec_msgs::CtrlInput>(ctrl_input_topic_, 10);
}

AsctecProc::~AsctecProc()
{
  ROS_INFO("Destroying AsctecProc"); 

}

void AsctecProc::llStatusCallback (const asctec_msgs::LLStatusPtr& ll_status_msg)
{
  engaged_ = ll_status_msg->flying;
}

void AsctecProc::stateCallback (const mav_msgs::StatePtr& state_msg)
{
  ROS_DEBUG("State callback, %d", state_msg->state);

  // detects changes in mav state, and engages/disengages motors

  if (prev_state_ == OFF && state_msg->state == IDLE)
    engageMotors();
  else if (prev_state_ == IDLE && state_msg->state == OFF)
    disengageMotors();

  prev_state_ = state_msg->state;
}

void AsctecProc::cmdYawCallback(const std_msgs::Float64ConstPtr& cmd_yaw)
{
  // translate from cmd_yaw [-1.0 to 1.0] to ctrl_yaw [-2047 .. 2047],
  int ctrl_yaw = (int)(cmd_yaw->data * ROS_TO_ASC_YAW);

  ROS_DEBUG ("\t\tCTRL_Yaw received: %d", ctrl_yaw);

  // change yaw in message and publish
  boost::mutex::scoped_lock(ctrl_mutex_);
  ctrl_input_msg_->yaw = ctrl_yaw;
  publishCtrlInputMsg();
}

void AsctecProc::cmdThrustCallback(const std_msgs::Float64ConstPtr& cmd_thrust)
{
  // translate from cmd_thrust [0.0 to 100.0] to ctrl_thrust [0 to 4091],
  int ctrl_thrust = (int)(cmd_thrust->data * ROS_TO_ASC_THRUST);

  ROS_DEBUG ("\t\tCTRL_Thrust received: %d", ctrl_thrust);

  // change thrust in message and publish
  boost::mutex::scoped_lock(ctrl_mutex_);
  ctrl_input_msg_->thrust = ctrl_thrust;
  publishCtrlInputMsg();
}

void AsctecProc::imuCalcDataCallback(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg)
{
  // publish imu message
  sensor_msgs::ImuPtr imuMsg = boost::make_shared<sensor_msgs::Imu>();
  createImuMsg (imuCalcDataMsg, imuMsg);
  imuPublisher_.publish(imuMsg);

  // publish unfiltered height message
  asctec_msgs::HeightPtr heightMsg = boost::make_shared<asctec_msgs::Height>();
  createHeightMsg (imuCalcDataMsg, heightMsg);
  heightPublisher_.publish(heightMsg);

  // publish filtered height message
  asctec_msgs::HeightPtr heightFilteredMsg = boost::make_shared<asctec_msgs::Height>();
  createHeightFilteredMsg (imuCalcDataMsg, heightFilteredMsg);
  heightFilteredPublisher_.publish(heightFilteredMsg);

/*
  printf("IMU: %f %f %f\n", imuCalcDataMsg->angle_roll * ASC_TO_ROS_ANGLE,
                            imuCalcDataMsg->angle_nick * ASC_TO_ROS_ANGLE,
                            imuCalcDataMsg->angle_yaw  * ASC_TO_ROS_ANGLE);
*/

/* //publish tf for testing

  btTransform t;

  btQuaternion orientation;

  orientation.setValue(imuMsg.orientation.x, imuMsg.orientation.y, imuMsg.orientation.z, imuMsg.orientation.w);
  t.setRotation (orientation);
  t.setOrigin (btVector3(0,0,0));

  tf::StampedTransform worldToOdomTf (t, ros::Time::now(), "navigation", "imu_raw");
  tfBroadcaster_.sendTransform (worldToOdomTf);
*/
}

void AsctecProc::createHeightMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                                       asctec_msgs::HeightPtr& heightMsg)
{
  // set header info
  heightMsg->header.stamp    = imuCalcDataMsg->header.stamp;
  heightMsg->header.frame_id = "imu";             // the frame seems arbitrary here

  heightMsg->height = imuCalcDataMsg->height_reference  * ASC_TO_ROS_HEIGHT;
  heightMsg->climb  = imuCalcDataMsg->dheight_reference * ASC_TO_ROS_HEIGHT;   
}

void AsctecProc::createHeightFilteredMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                                               asctec_msgs::HeightPtr& heightMsg)
{
  // set header info
  heightMsg->header.stamp    = imuCalcDataMsg->header.stamp;
  heightMsg->header.frame_id = "imu";             // the frame seems arbitrary here

  heightMsg->height = imuCalcDataMsg->height  * ASC_TO_ROS_HEIGHT;
  heightMsg->climb  = imuCalcDataMsg->dheight * ASC_TO_ROS_HEIGHT;   
}

void AsctecProc::createImuMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                                    sensor_msgs::ImuPtr& imuMsg)
{
  // set header info
  imuMsg->header.stamp    = imuCalcDataMsg->header.stamp;
  imuMsg->header.frame_id = "imu";

  // copy over linear acceleration
  imuMsg->linear_acceleration.x = imuCalcDataMsg->acc_x_calib * ASC_TO_ROS_ACC;
  imuMsg->linear_acceleration.y = imuCalcDataMsg->acc_y_calib * ASC_TO_ROS_ACC;
  imuMsg->linear_acceleration.z = imuCalcDataMsg->acc_z_calib * ASC_TO_ROS_ACC;

/* // Uncomment these if you use covariances
  // define linear acceleration variance
  imuMsg->linear_acceleration_covariance[0] = 1.0;
  imuMsg->linear_acceleration_covariance[1] = 0.0;
  imuMsg->linear_acceleration_covariance[2] = 0.0;
  imuMsg->linear_acceleration_covariance[3] = 0.0;
  imuMsg->linear_acceleration_covariance[4] = 1.0;
  imuMsg->linear_acceleration_covariance[5] = 0.0;
  imuMsg->linear_acceleration_covariance[6] = 0.0;
  imuMsg->linear_acceleration_covariance[7] = 0.0;
  imuMsg->linear_acceleration_covariance[8] = 1.0;
*/
  // copy over angular_velocity
  imuMsg->angular_velocity.x = imuCalcDataMsg->angvel_roll * ASC_TO_ROS_ANGVEL * -1.0;
  imuMsg->angular_velocity.y = imuCalcDataMsg->angvel_nick * ASC_TO_ROS_ANGVEL;
  imuMsg->angular_velocity.z = imuCalcDataMsg->angvel_yaw  * ASC_TO_ROS_ANGVEL * -1.0; 

/* // Uncomment these if you use covariances
  // define angular_velocity variance
  imuMsg->angular_velocity_covariance[0] = 1.0;
  imuMsg->angular_velocity_covariance[1] = 0.0;
  imuMsg->angular_velocity_covariance[2] = 0.0;
  imuMsg->angular_velocity_covariance[3] = 0.0;
  imuMsg->angular_velocity_covariance[4] = 1.0;
  imuMsg->angular_velocity_covariance[5] = 0.0;
  imuMsg->angular_velocity_covariance[6] = 0.0;
  imuMsg->angular_velocity_covariance[7] = 0.0;
  imuMsg->angular_velocity_covariance[8] = 1.0;
*/

  // calculate quaternion orientation
  btQuaternion orientation;
  orientation.setRPY(imuCalcDataMsg->angle_roll * ASC_TO_ROS_ANGLE * -1.0,
                     imuCalcDataMsg->angle_nick * ASC_TO_ROS_ANGLE,
                     imuCalcDataMsg->angle_yaw  * ASC_TO_ROS_ANGLE * -1.0);

  imuMsg->orientation.x = orientation.getX();
  imuMsg->orientation.y = orientation.getY();
  imuMsg->orientation.z = orientation.getZ();
  imuMsg->orientation.w = orientation.getW();
}

void AsctecProc::engageMotors()
{
  // set the stick to lower left, wait for motors to engage, 
  // and reset stick

  ROS_DEBUG ("\tEngaging motors...");

  boost::mutex::scoped_lock(ctrl_mutex_);

  ctrl_input_msg_->thrust = 0;
  ctrl_input_msg_->yaw = -2047;
  publishCtrlInputMsg();

  while(!engaged_)
  {
    ros::Duration(0.1).sleep();
    ctrl_input_msg_->thrust = 0;
    ctrl_input_msg_->yaw = -2047;
    publishCtrlInputMsg();
  }

  ctrl_input_msg_->thrust = 0;
  ctrl_input_msg_->yaw = 0;
  publishCtrlInputMsg();

  ROS_DEBUG("\tDone engaging motors.");
}

void AsctecProc::disengageMotors()
{
  // set the stick to lower left, wait for motors to disengage, 
  // and reset stick

  ROS_DEBUG ("\tDisengaging motors...");

  boost::mutex::scoped_lock(ctrl_mutex_);

  ctrl_input_msg_->thrust = 0;
  ctrl_input_msg_->yaw = -2047;
  publishCtrlInputMsg();

  while(engaged_)
  {
    ros::Duration(0.1).sleep();
    ctrl_input_msg_->thrust = 0;
    ctrl_input_msg_->yaw = -2047;
    publishCtrlInputMsg();
  }

  ctrl_input_msg_->thrust = 0;
  ctrl_input_msg_->yaw = 0;
  publishCtrlInputMsg();

  ROS_DEBUG("\tDone disengaging motors.");
}

void AsctecProc::publishCtrlInputMsg()
{
  ROS_DEBUG("Publishing...");

  // update checksum and timestamp, and publish
  ctrl_input_msg_->chksum = ctrl_input_msg_->roll + ctrl_input_msg_->pitch  + 
                            ctrl_input_msg_->yaw  + ctrl_input_msg_->thrust + 
                            ctrl_input_msg_->ctrl - 21846;
  ctrl_input_msg_->header.stamp = ros::Time::now();
  ctrl_input_publisher_.publish(ctrl_input_msg_);
}

} // end namespace asctec
