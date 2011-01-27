#ifndef SERIAL_READER_COMMON_H
#define SERIAL_READER_COMMON_H

//packet descriptors
#define PD_IMURAWDATA       0x01
#define PD_LLSTATUS        	0x02
#define PD_IMUCALCDATA      0x03
#define PD_HLSTATUS        	0x04

#define PD_CTRLOUT			    0x11
#define PD_FLIGHTPARAMS     0x12
#define PD_CTRLCOMMANDS		  0x13
#define PD_CTRLINTERNAL		  0x14
#define PD_RCDATA       	  0x15
#define PD_CTRLSTATUS	    	0x16

#define PD_WAYPOINT     	  0x20
#define PD_CURRENTWAY   	  0x21
#define PD_NMEADATA       	0x22
#define PD_GPSDATA			    0x23

#define PD_CAMERACOMMANDS 	0x30

// CONVERSION FACTORS

const double PEL_TO_ROS_ANGLE  = (1.0 /  1000.0) * 3.14159265 / 180.0; // converts to rad
const double PEL_TO_ROS_ANGVEL = (1.0 /    64.8) * 3.14159265 / 180.0; // convetts to rad/s
const double PEL_TO_ROS_ACC    = (1.0 / 10000.0) * 9.81;               // converts to m/s^s
const double PEL_TO_ROS_HEIGHT = (1.0 /  1000.0);                      // converts to m

struct IMU_CALCDATA {
  //angles derived by integration of gyro_outputs, drift compensated by data fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
  int angle_nick;
  int angle_roll;
  int angle_yaw;

  //angular velocities, raw values [16 bit], bias free, in 0.0154 °/s (=> 64.8 = 1 °/s)
  int angvel_nick;
  int angvel_roll;
  int angvel_yaw;

  //acc-sensor outputs, calibrated: -10000..+10000 = -1g..+1g
  short acc_x_calib;
  short acc_y_calib;
  short acc_z_calib;

  //horizontal / vertical accelerations: -10000..+10000 = -1g..+1g
  short acc_x;
  short acc_y;
  short acc_z;

  //reference angles derived by accelerations only: -90000..+90000; 1000 = 1 degree
  int acc_angle_nick;
  int acc_angle_roll;

  //total acceleration measured (10000 = 1g)
  int acc_absolute_value;

  //magnetic field sensors output, offset free and scaled; units not determined, as only the direction of the field vector is taken into account
  int Hx;
  int Hy;
  int Hz;

  //compass reading: angle reference for angle_yaw: 0..360000; 1000 = 1 degree
  int mag_heading;

  //pseudo speed measurements: integrated accelerations, pulled towards zero; units unknown; used for short-term position stabilization
  int speed_x;
  int speed_y;
  int speed_z;

  //height in mm (after data fusion)
  int height;

  //diff. height in mm/s (after data fusion)
  int dheight;

  //diff. height measured by the pressure sensor [mm/s]
  int dheight_reference;

  //height measured by the pressure sensor [mm]
  int height_reference;
};

#endif

