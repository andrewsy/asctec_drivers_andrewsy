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
//#include <tf/transform_broadcaster.h>

namespace asctec
{
class AsctecProc
{
  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber cmd_thrust_subscriber_;
    ros::Subscriber cmd_yaw_subscriber_;

    ros::Subscriber ll_status_subscriber_;

    ros::Subscriber imuCalcDataSubscriber_;
    ros::Publisher  imuPublisher_;
    ros::Publisher  heightPublisher_;
    ros::Publisher  heightFilteredPublisher_;
    ros::Publisher  ctrl_input_publisher_;

    ros::Subscriber  state_subscriber_;
    ros::Subscriber  estop_subscriber_;

    //tf::TransformBroadcaster tfBroadcaster_;

    boost::mutex ctrl_mutex_;
    asctec_msgs::CtrlInputPtr ctrl_input_msg_;        // periodically sent to autopilot
    asctec_msgs::CtrlInputPtr ctrl_input_toggle_msg_; // stick to the lower left
    asctec_msgs::CtrlInputPtr ctrl_input_zero_msg_;   // zero message (sticks centered)

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

    // **** state variables

    bool engaged_;
    int prev_state_;

    // **** member functions

    void initializeParams();
    void assembleCtrlCommands();

    void cmdThrustCallback(const std_msgs::Float64ConstPtr& cmd_thrust);
    void cmdYawCallback   (const std_msgs::Float64ConstPtr& cmd_yaw);
    void stateCallback    (const mav_msgs::StatePtr&        state_msg);

    void llStatusCallback (const asctec_msgs::LLStatusPtr& ll_status_msg);


    void imuCalcDataCallback(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg);

    void createImuMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                            sensor_msgs::ImuPtr& imuMsg);

    void createHeightMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                               mav_msgs::HeightPtr& heightMsg);

    void createHeightFilteredMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                                       mav_msgs::HeightPtr& heightMsg);

    void engageMotors();
    void disengageMotors();
    void publishCtrlInputMsg();

  public:

    AsctecProc(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~AsctecProc();

};

} // end namespace asctec

#endif //ASCTEC_PROC_ASCTEC_PROC_H
