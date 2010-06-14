#ifndef SERIAL_READER_SERIAL_READER_H
#define SERIAL_READER_SERIAL_READER_H

#include <stdio.h>
#include <sys/termios.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <ros/ros.h>

#include "serial_reader/common.h"

class SerialReader
{
  private: 
  
    static unsigned short crc_update (unsigned short crc, unsigned char data);
    static unsigned short crc16(void* data, unsigned short cnt);	

  public:

	  FILE* dev, *output;
	  int* scan;
	  bool status;
	  int pt[800];
	  int counter;
	
	  speed_t bitrate(int);
	  int setupSerial(int, int);
	  FILE* openSerial(const char*, int);

    bool getIMUCalcdata(IMU_CALCDATA& data);
    bool getPacket(char spacket[1024], unsigned char &packet_type, unsigned short &packet_crc);
	
	  SerialReader();
	  virtual ~SerialReader();
};

#endif

