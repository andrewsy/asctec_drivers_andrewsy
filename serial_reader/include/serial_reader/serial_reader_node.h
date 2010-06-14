#ifndef SERIAL_READER_SERIAL_READER_NODE_H
#define SERIAL_READER_SERIAL_READER_NODE_H

#include <algorithm>

#include <btBulletDynamicsCommon.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include "serial_reader/Height.h"
#include "serial_reader/common.h"
#include "serial_reader/serial_reader.h"

#include <tf/transform_broadcaster.h>

class SerialReaderNode
{
  private: 

		ros::Publisher imuPublisher_;
		ros::Publisher heightPublisher_;
		ros::Publisher heightRefPublisher_;
		ros::Publisher dheightPublisher_;
		ros::Publisher dheightRefPublisher_;
  	ros::Timer timer_;
    ros::NodeHandle nh_;

    // transfor boradcasters
    tf::TransformBroadcaster odomBroadcaster_;

    SerialReader serialReader_;

    void createImuMessage(const IMU_CALCDATA& serialIMUCalcdata, sensor_msgs::Imu& imuMessage);
    void createHeightMessage(const IMU_CALCDATA& serialIMUCalcdata, serial_reader::Height& heightMessage);
    void createHeightRefMessage(const IMU_CALCDATA& serialIMUCalcdata, serial_reader::Height& heightMessage);

    void createDHeightMessage(const IMU_CALCDATA& serialIMUCalcdata, serial_reader::Height& heightMessage);
    void createDHeightRefMessage(const IMU_CALCDATA& serialIMUCalcdata, serial_reader::Height& heightMessage);

  public:

    SerialReaderNode();
    ~SerialReaderNode();

    void spin(const ros::TimerEvent& e);
};

#endif

