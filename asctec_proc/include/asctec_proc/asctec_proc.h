#ifndef ASCTEC_PROC_ASCTEC_PROC_H
#define ASCTEC_PROC_ASCTEC_PROC_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <mav_msgs/State.h>
#include <asctec_msgs/IMUCalcData.h>
#include <sensor_msgs/Imu.h>
#include <asctec_msgs/Height.h>
#include <asctec_msgs/CtrlInput.h>
#include <asctec_msgs/LLStatus.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

const std::string rawdata_namespace_    = "asctec";
const std::string procdata_namespace_   = "mav";

const std::string cmd_thrust_topic_    = "cmd_thrust";
const std::string cmd_yaw_topic_       = "cmd_yaw";

const std::string ctrl_input_topic_    = "CTRL_INPUT";
const std::string ll_status_topic_     = "LL_STATUS";

const std::string imuCalcDataTopic_    = "IMU_CALCDATA";
const std::string imuTopic_            = "imu";
const std::string heightTopic_         = "pressure_height";
const std::string heightFilteredTopic_ = "pressure_height_filtered";

const std::string state_topic_       = "state";

const double ASC_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
const double ASC_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
const double ASC_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
const double ASC_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m

// from asctec CtrlInput definitions
const double ROS_TO_ASC_THRUST = (4095 / 100.0);      // convertrs from % to thrust counts
const double ROS_TO_ASC_YAW    = 2047.0;              // converts from [-1, 1] to yaw counts

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

    tf::TransformBroadcaster tfBroadcaster_;

    boost::mutex ctrl_mutex_;
    asctec_msgs::CtrlInputPtr ctrl_input_msg_;

    bool engaged_;
    int prev_state_;

    void cmdThrustCallback(const std_msgs::Float64ConstPtr& cmd_thrust);
    void cmdYawCallback   (const std_msgs::Float64ConstPtr& cmd_yaw);
    void stateCallback    (const mav_msgs::StatePtr&        state_msg);

    void llStatusCallback (const asctec_msgs::LLStatusPtr& ll_status_msg);


    void imuCalcDataCallback(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg);

    void createImuMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                            sensor_msgs::ImuPtr& imuMsg);

    void createHeightMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                               asctec_msgs::HeightPtr& heightMsg);

    void createHeightFilteredMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                                       asctec_msgs::HeightPtr& heightMsg);

    void engageMotors();
    void disengageMotors();
    void publishCtrlInputMsg();

  public:

    AsctecProc(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~AsctecProc();

};

} // end namespace asctec

#endif //ASCTEC_PROC_ASCTEC_PROC_H
