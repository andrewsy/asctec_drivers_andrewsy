#ifndef ASCTEC_PROC_ASCTEC_PROC_H
#define ASCTEC_PROC_ASCTEC_PROC_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/common.h>
#include <mav_msgs/State.h>
#include <mav_msgs/Height.h>
#include <sensor_msgs/Imu.h>
#include <asctec_msgs/common.h>
#include <asctec_msgs/IMUCalcData.h>
#include <asctec_msgs/CtrlInput.h>
#include <asctec_msgs/LLStatus.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_datatypes.h>

#include "mav_msgs/Engage.h"
#include "mav_msgs/GetEngaged.h"

namespace asctec
{

// **** conversion units

const double ASC_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
const double ASC_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
const double ASC_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
const double ASC_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m

// from asctec CtrlInput definitions
const double ROS_TO_ASC_THRUST   = 4095.0;          // converts from [ 0, 1] to thrust stick counts
const double ROS_TO_ASC_ROLL     = 2047.0;          // converts from [-1, 1] to roll stick counts
const double ROS_TO_ASC_PITCH    = 2047.0;          // converts from [-1, 1] to pitch stick counts

// Per email from AscTec,
// """The standard parameter for K_stick_yaw is 120, resulting in a maximum rate of
// 254.760 degrees per second = 4.43 rad/s. I.e. a 360° turn takes about 1.5 seconds."""
const double ROS_TO_ASC_YAW_RATE = 2047.0/4.43;  // converts from rad/s to yaw_rate stick counts

class AsctecProc
{
  private:

    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber cmd_thrust_subscriber_;
    ros::Subscriber cmd_roll_subscriber_;
    ros::Subscriber cmd_pitch_subscriber_;
    ros::Subscriber cmd_yaw_subscriber_;
    ros::Subscriber ll_status_subscriber_;
    ros::Subscriber imu_calcdata_subscriber_;
    ros::Subscriber state_subscriber_;
    ros::Subscriber estop_subscriber_;

    ros::Publisher imu_publisher_;
    ros::Publisher height_publisher_;
    ros::Publisher height_filtered_publisher_;
    ros::Publisher ctrl_input_publisher_;

    ros::ServiceServer engage_srv_;
    ros::ServiceServer get_engaged_srv_;

    // **** state variables

    bool state_;    // state is currently being modified

    boost::mutex state_mutex_;

    asctec_msgs::CtrlInputPtr ctrl_input_msg_;        // periodically sent to autopilot
    asctec_msgs::CtrlInputPtr ctrl_input_toggle_msg_; // stick to the lower left
    asctec_msgs::CtrlInputPtr ctrl_input_zero_msg_;   // zero message (sticks centered)

    sensor_msgs::ImuPtr imu_msg_;
    mav_msgs::HeightPtr height_msg_;
    mav_msgs::HeightPtr height_filtered_msg_;

    bool engaged_;
    bool engaging_;

    int prev_state_;

    // **** parameters

    bool enable_ctrl_thrust_;
    bool enable_ctrl_roll_;
    bool enable_ctrl_pitch_;
    bool enable_ctrl_yaw_;

    bool enable_state_changes_;   // if true, monitor state, turn on/iff 

    int max_ctrl_thrust_;   // max output - in asctec units
    int max_ctrl_roll_; 
    int max_ctrl_pitch_;
    int max_ctrl_yaw_;

    // **** member functions

    void initializeParams();
    void assembleCtrlCommands();

    void cmdThrustCallback(const std_msgs::Float64ConstPtr& cmd_thrust_msg);
    void cmdRollCallback  (const std_msgs::Float64ConstPtr& cmd_roll_msg);
    void cmdPitchCallback (const std_msgs::Float64ConstPtr& cmd_pitch_msg);
    void cmdYawCallback   (const std_msgs::Float64ConstPtr& cmd_yaw_rate_msg);
    //void stateCallback    (const mav_msgs::StatePtr&        state_msg);
    void llStatusCallback (const asctec_msgs::LLStatusPtr& ll_status_msg);
    void imuCalcDataCallback(const asctec_msgs::IMUCalcDataConstPtr& imu_calcdata_msg);

    void createImuMsg           (const asctec_msgs::IMUCalcDataConstPtr& imu_calcdata_msg);
    void createHeightMsg        (const asctec_msgs::IMUCalcDataConstPtr& imu_calcdata_msg);
    void createHeightFilteredMsg(const asctec_msgs::IMUCalcDataConstPtr& imu_calcdata_msg);

    void engageMotors();
    void disengageMotors();
    void publishCtrlInputMsg();

    bool engage(mav_msgs::Engage::Request  &req,
                mav_msgs::Engage::Response &res);
    bool getEngaged(mav_msgs::GetEngaged::Request  &req,
                    mav_msgs::GetEngaged::Response &res);

  public:

    AsctecProc(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~AsctecProc();

};

} // end namespace asctec

#endif //ASCTEC_PROC_ASCTEC_PROC_H
