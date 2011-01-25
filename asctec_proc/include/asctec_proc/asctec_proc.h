#ifndef ASCTEC_PROC_ASCTEC_PROC_H
#define ASCTEC_PROC_ASCTEC_PROC_H

#include <ros/ros.h>
#include <asctec_msgs/IMUCalcData.h>
#include <sensor_msgs/Imu.h>
#include <asctec_msgs/Height.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

const std::string subscribe_namespace_ = "asctec_raw";
const std::string publish_namespace_   = "asctec_proc";

const std::string imuCalcDataTopic_    = "IMU_CALCDATA";
const std::string imuTopic_            = "imu";
const std::string heightTopic_         = "pressure_height";
const std::string heightFilteredTopic_ = "pressure_height_filtered";

const double ASC_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
const double ASC_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
const double ASC_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
const double ASC_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m

namespace asctec
{
class AsctecProc
{
  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber imuCalcDataSubscriber_;
    ros::Publisher  imuPublisher_;
    ros::Publisher  heightPublisher_;
    ros::Publisher  heightFilteredPublisher_;
    tf::TransformBroadcaster tfBroadcaster_;

    void imuCalcDataCallback(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg);

    void createImuMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                            sensor_msgs::ImuPtr& imuMsg);

    void createHeightMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                               asctec_msgs::HeightPtr& heightMsg);

    void createHeightFilteredMsg(const asctec_msgs::IMUCalcDataConstPtr& imuCalcDataMsg,
                                       asctec_msgs::HeightPtr& heightMsg);

  public:

    AsctecProc(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~AsctecProc();

};

} // end namespace asctec

#endif //ASCTEC_PROC_ASCTEC_PROC_H
