#include "serial_reader/calibrate.h"

int main(int argc, char** argv)
{
	int iterations = atoi(argv[1]);

  // set up stat data instances

  statData angle_roll (iterations, "angle_roll", "rad");
  statData angle_pitch(iterations, "angle_pitch", "rad");
  statData angle_yaw  (iterations, "angle_yaw", "rad");

  statData angvel_roll (iterations, "angvel_roll", "rad");
  statData angvel_pitch(iterations, "angvel_pitch", "rad");
  statData angvel_yaw  (iterations, "angvel_yaw", "rad");

  statData acc_x(iterations, "acc_x", "m/s^2");
  statData acc_y(iterations, "acc_y", "m/s^2");
  statData acc_z(iterations, "acc_z", "m/s^2");

  statData acc_x_calib(iterations, "acc_x_calib", "m/s^2");
  statData acc_y_calib(iterations, "acc_y_calib", "m/s^2");
  statData acc_z_calib(iterations, "acc_z_calib", "m/s^2");

  statData height          (iterations, "height", "m");
  statData height_reference(iterations, "height_ref", "m");

	// **** set up serial reader

	SerialReader serialReader;
	IMU_CALCDATA serialIMUCalcdata;

	int result = initSerial(&serialReader);
	if (result != 0) return result;

	// **** collect readings

	printf("Collecting %d readings:\n", iterations);
	printf("\treading .....");

	int i = 0;
	while (i < iterations)
	{
		if(serialReader.getIMUCalcdata(serialIMUCalcdata))
		{
			printf("\b\b\b\b\b%5d", i+1);

			// log orientation

			angle_roll.val[i]  = serialIMUCalcdata.angle_roll * PEL_TO_ROS_ANGLE;
			angle_pitch.val[i] = serialIMUCalcdata.angle_nick * PEL_TO_ROS_ANGLE;
			angle_yaw.val[i]   = serialIMUCalcdata.angle_yaw  * PEL_TO_ROS_ANGLE;

			// log angular velocities

			angvel_roll.val[i]  = serialIMUCalcdata.angvel_roll * PEL_TO_ROS_ANGVEL;
			angvel_pitch.val[i] = serialIMUCalcdata.angvel_nick * PEL_TO_ROS_ANGVEL;
			angvel_yaw.val[i]   = serialIMUCalcdata.angvel_yaw  * PEL_TO_ROS_ANGVEL;

			// log accelerations 

			acc_x_calib.val[i] = serialIMUCalcdata.acc_x_calib * PEL_TO_ROS_ACC;
			acc_y_calib.val[i] = serialIMUCalcdata.acc_y_calib * PEL_TO_ROS_ACC;
			acc_z_calib.val[i] = serialIMUCalcdata.acc_z_calib * PEL_TO_ROS_ACC;

			acc_x.val[i] = serialIMUCalcdata.acc_x * PEL_TO_ROS_ACC;
			acc_y.val[i] = serialIMUCalcdata.acc_y * PEL_TO_ROS_ACC;
			acc_z.val[i] = serialIMUCalcdata.acc_z * PEL_TO_ROS_ACC;

      // log heights

      height.val[i]           = serialIMUCalcdata.height           * PEL_TO_ROS_HEIGHT;
      height_reference.val[i] = serialIMUCalcdata.height_reference * PEL_TO_ROS_HEIGHT;

			i++;
		}

		usleep(50000);
  }

	printf("\nReadings collected.\n");

  // **** update stat data

	angle_roll.update();
	angle_pitch.update();
	angle_yaw.update();

	angvel_roll.update();
	angvel_pitch.update();
	angvel_yaw.update();

	acc_x.update();
	acc_y.update();
	acc_z.update();

	acc_x_calib.update();
	acc_y_calib.update();
	acc_z_calib.update();

	height.update();
	height_reference.update();

  // ***** print
  
  angle_roll.printHeader();

	angle_roll.print();
	angle_pitch.print();
	angle_yaw.print();

	angvel_roll.print();
	angvel_pitch.print();
	angvel_yaw.print();

	acc_x.print();
	acc_y.print();
	acc_z.print();

	acc_x_calib.print();
	acc_y_calib.print();
	acc_z_calib.print();

	height.print();
	height_reference.print();
}

int initSerial(SerialReader* serialReader)
{
  ROS_INFO("Initializing serial port...");

  // set default paramters
  int baud;
  std::string port;
  std::string logfile;

  ros::NodeHandle nh_;
  nh_.param("baud", baud, 57600);
  nh_.param<std::string>("port",    port,    "/dev/ttyUSB0");
  nh_.param<std::string>("logfile", logfile, "log.txt");

  // open log file

  serialReader->output=fopen(logfile.c_str(), "w");
  if(serialReader->output==NULL)
  {
	  ROS_ERROR("Could not open log file %s", logfile.c_str());
	  return 1;
  }

  // establish a connection with pelican port

  serialReader->dev = serialReader->openSerial(port.c_str(), baud);
  if(serialReader->dev == NULL)
  {
	  ROS_ERROR("Could not open serial port %s", port.c_str());
	  return 1;
  }

  ROS_INFO("Successfully connected to %s, Baudrate %d\n", port.c_str(), baud);

  return 0;
}

