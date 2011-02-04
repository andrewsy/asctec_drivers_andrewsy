#ifndef ASCTEC_PROC_ASCTEC_PROC_H
#define ASCTEC_PROC_ASCTEC_PROC_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/State.h>
#include <mav_msgs/Height.h>
#include <sensor_msgs/Imu.h>
#include <asctec_msgs/IMUCalcData.h>
#include <asctec_msgs/CtrlInput.h>
#include <asctec_msgs/LLStatus.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_datatypes.h>
//#include <tf/transform_broadcaster.h>

const std::string rawdata_namespace_    = "asctec";
const std::string procdata_namespace_   = "mav";

// **** max namespace topics

const std::string cmd_thrust_topic_      = "cmd_thrust";
const std::string cmd_yaw_topic_         = "cmd_yaw";
const std::string imu_topic_             = "imu";
const std::string height_topic_          = "pressure_height";
const std::string height_filtered_topic_ = "pressure_height_filtered";
const std::string state_topic_           = "state";

// **** asctec namespace topics

const std::string ctrl_input_topic_    = "CTRL_INPUT";
const std::string ll_status_topic_     = "LL_STATUS";
const std::string imu_calcdata_topic_   = "IMU_CALCDATA";

// **** conversion units

const double ASC_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
const double ASC_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
const double ASC_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
const double ASC_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m

// from asctec CtrlInput definitions
const double ROS_TO_ASC_THRUST = 4095.0;      // converts from [ 0, 1] to thrust counts
const double ROS_TO_ASC_YAW    = 2047.0;      // converts from [-1, 1] to yaw counts

enum MAVState {OFF = 0, IDLE = 1, FLYING = 2};

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
    asctec_msgs::CtrlInputPtr ctrl_input_msg_;

    // **** parameters

    bool enable_ctrl_thrust_;
    bool enable_ctrl_roll_;
    bool enable_ctrl_pitch_;
    bool enable_ctrl_yaw_;

    int max_ctrl_thrust_;   // max output - in asctec units
    int max_ctrl_roll_; 
    int max_ctrl_pitch_;
    int max_ctrl_yaw_;

    // **** state variables

    bool engaged_;
    int prev_state_;

    // **** member functions

    void initializeParams();

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
