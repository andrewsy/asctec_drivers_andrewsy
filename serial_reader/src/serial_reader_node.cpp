#include "serial_reader/serial_reader_node.h"

SerialReaderNode::SerialReaderNode()
{
	ROS_INFO("Starting SerialReaderNode"); 

	// **** set up ROS **************************

  // set default paramters
  double freq;
  int baud;
  std::string port;
  std::string logfile;

  nh_.param("freq", freq, 10.0);
  nh_.param("baud", baud, 57600);
  nh_.param<std::string>("port",    port,    "/dev/ttyUSB0");
  nh_.param<std::string>("logfile", logfile, "log.txt");
	
  const char* imuTopicName_       = "imu";

  const char* heightTopicName_    = "height";
  const char* heightRefTopicName_ = "height_ref";
  const char* dheightTopicName_    = "dheight";
  const char* dheightRefTopicName_ = "dheight_ref";

	// set up timer
  timer_ = nh_.createTimer(ros::Duration(1.0/std::max(freq, 1.0)), &SerialReaderNode::spin, this);

  // advertise imu publisher
  imuPublisher_ = nh_.advertise<sensor_msgs::Imu>(imuTopicName_, 10);
	ROS_INFO("Publishing IMU data on topic: %s", imuTopicName_); 

  // advertise height publisher
  heightPublisher_ = nh_.advertise<serial_reader::Height>(heightTopicName_, 10);
	ROS_INFO("Publishing height data on topic: %s", heightTopicName_); 

  // advertise height ref publisher
  heightRefPublisher_ = nh_.advertise<serial_reader::Height>(heightRefTopicName_, 10);
	ROS_INFO("Publishing height_ref data on topic: %s", heightRefTopicName_); 

  // advertise dheight publisher
  dheightPublisher_ = nh_.advertise<serial_reader::Height>(dheightTopicName_, 10);
	ROS_INFO("Publishing dheight data on topic: %s", dheightTopicName_); 

  // advertise height ref publisher
  dheightRefPublisher_ = nh_.advertise<serial_reader::Height>(dheightRefTopicName_, 10);
	ROS_INFO("Publishing dheight_ref data on topic: %s", dheightRefTopicName_); 

  // **** set up serial port *****************

  ROS_INFO("Initializing serial port...");

  // open log file

	serialReader_.output=fopen(logfile.c_str(),"w");
	if(serialReader_.output==NULL)
	{
		ROS_ERROR("Could not open log file %s", logfile.c_str());
	}
  else
  {
	  // establish a connection with pelican port

	  serialReader_.dev = serialReader_.openSerial(port.c_str(), baud);
	  if(serialReader_.dev == NULL)
		  ROS_ERROR("Could not open serial port %s", port.c_str());
    else
    	ROS_INFO("Successfully connected to %s, Baudrate %d\n", port.c_str(), baud);
  }
}

SerialReaderNode::~SerialReaderNode()
{
    // TODO - close serial port
}

void SerialReaderNode::spin(const ros::TimerEvent& e)
{
  IMU_CALCDATA serialIMUCalcdata;

  sensor_msgs::Imu  imuMessage;
  imuMessage.header.frame_id = "imu";

  serial_reader::Height heightMessage;
  heightMessage.header.frame_id = "height";
  
  serial_reader::Height heightRefMessage;
  heightRefMessage.header.frame_id = "height";

  // TODO - publish covariances in the message

  if(serialReader_.getIMUCalcdata(serialIMUCalcdata))
  {
    ROS_DEBUG("publishing serial data");

    // create and publish imu message

    createImuMessage(serialIMUCalcdata, imuMessage);
    imuPublisher_.publish(imuMessage);

    // create and publish height message

    createHeightMessage(serialIMUCalcdata, heightMessage);
    heightPublisher_.publish(heightMessage);

    // create and publish height ref message

    createHeightRefMessage(serialIMUCalcdata, heightRefMessage);
    heightRefPublisher_.publish(heightRefMessage);

    // create and publish dheight message

    createDHeightMessage(serialIMUCalcdata, heightMessage);
    dheightPublisher_.publish(heightMessage);

    // create and publish dheight ref message

    createDHeightRefMessage(serialIMUCalcdata, heightRefMessage);
    dheightRefPublisher_.publish(heightRefMessage);

/*
    // **** publish odometry transform

    btVector3 pos;
    pos.setValue(0, 0, serialIMUCalcdata.dheight * PEL_TO_ROS_HEIGHT);

    btQuaternion rot;
    rot.setRPY(-serialIMUCalcdata.angle_roll * PEL_TO_ROS_ANGLE, 
                serialIMUCalcdata.angle_nick * PEL_TO_ROS_ANGLE, 
               -serialIMUCalcdata.angle_nick * PEL_TO_ROS_ANGLE);

    tf::Transform transform;
    transform.setOrigin(pos);
    transform.setRotation(rot);

    odomBroadcaster_.sendTransform(tf::StampedTransform(
      transform, ros::Time::now(), "odom", "base_link"));
*/
  }
  else
  {
    ROS_ERROR("failed to read serial data");
  }
}

void SerialReaderNode::createImuMessage(const IMU_CALCDATA& serialIMUCalcdata,
                                        sensor_msgs::Imu& imuMessage)
{
  imuMessage.header.stamp = ros::Time::now(); 

  // get orientation

  btQuaternion orientation;

	double roll  = serialIMUCalcdata.angle_roll * PEL_TO_ROS_ANGLE;
	double pitch = serialIMUCalcdata.angle_nick * PEL_TO_ROS_ANGLE;
	double yaw   = serialIMUCalcdata.angle_yaw  * PEL_TO_ROS_ANGLE;

  orientation.setRPY(roll, pitch, yaw);

  imuMessage.orientation.x = orientation.getX();
  imuMessage.orientation.y = orientation.getY();
  imuMessage.orientation.z = orientation.getZ();
  imuMessage.orientation.w = orientation.getW();

	// get angular velocities

	imuMessage.angular_velocity.x = serialIMUCalcdata.angvel_roll * PEL_TO_ROS_ANGVEL;
	imuMessage.angular_velocity.y = serialIMUCalcdata.angvel_nick * PEL_TO_ROS_ANGVEL;
	imuMessage.angular_velocity.z = serialIMUCalcdata.angvel_yaw  * PEL_TO_ROS_ANGVEL;

	// get accelerations 

	imuMessage.linear_acceleration.x = serialIMUCalcdata.acc_x * PEL_TO_ROS_ACC;
	imuMessage.linear_acceleration.y = serialIMUCalcdata.acc_y * PEL_TO_ROS_ACC;
	imuMessage.linear_acceleration.z = serialIMUCalcdata.acc_z * PEL_TO_ROS_ACC;
}
    
void SerialReaderNode::createHeightMessage(const IMU_CALCDATA& serialIMUCalcdata,
                                           serial_reader::Height& heightMessage)
{
    heightMessage.height = serialIMUCalcdata.height * PEL_TO_ROS_HEIGHT;
    heightMessage.header.stamp = ros::Time::now(); 
}             

void SerialReaderNode::createHeightRefMessage(const IMU_CALCDATA& serialIMUCalcdata,
                                              serial_reader::Height& heightMessage)
{
    heightMessage.height = serialIMUCalcdata.height_reference * PEL_TO_ROS_HEIGHT;
    heightMessage.header.stamp = ros::Time::now(); 
}                                

void SerialReaderNode::createDHeightMessage(const IMU_CALCDATA& serialIMUCalcdata,
                                           serial_reader::Height& heightMessage)
{
    heightMessage.height = serialIMUCalcdata.dheight * PEL_TO_ROS_HEIGHT;
    heightMessage.header.stamp = ros::Time::now(); 
}             

void SerialReaderNode::createDHeightRefMessage(const IMU_CALCDATA& serialIMUCalcdata,
                                              serial_reader::Height& heightMessage)
{
    heightMessage.height = serialIMUCalcdata.dheight_reference * PEL_TO_ROS_HEIGHT;
    heightMessage.header.stamp = ros::Time::now(); 
}    
