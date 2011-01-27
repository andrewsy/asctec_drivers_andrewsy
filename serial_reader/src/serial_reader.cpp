#include "serial_reader/serial_reader.h"

SerialReader::SerialReader()
{
	status = false;
}

SerialReader::~SerialReader()
{

}

speed_t SerialReader::bitrate(int aBR)
{
  switch(aBR)
    {
    case 9600:
      return B9600;

    case 19200:
      return B19200;

    case 38400:
      return B38400;

    case 57600:
      return B57600;

    case 115200:
      return B115200;

    case 230400:
      return B230400;

    default: // invalid bitrate
      return B0;
    }
}

int SerialReader::setupSerial(int aFd, int aSpeed)
{
  int speed;
  struct termios tio;

  speed=bitrate(aSpeed);
  if(speed==0)
    return 0;  // invalid bitrate

  tcgetattr(aFd,&tio);

  cfsetispeed(&tio,speed); // bitrate
  cfsetospeed(&tio,speed); // bitrate

  // I_FLAGS
  tio.c_iflag = 0;
  tio.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
  tio.c_iflag |= IGNBRK;
;
  // O_FLAGS
  tio.c_oflag = 0;
  tio.c_oflag &= ~(OPOST | ONLCR);

  // C_FLAGS
  tio.c_cflag=(tio.c_cflag & ~CSIZE) | CS8; // data bits = 8bit
  tio.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);

  // L_FLAGS
  tio.c_lflag = 0;
  tio.c_lflag |= NOFLSH;
  tio.c_lflag &= ~( ISIG | IEXTEN | ICANON | ECHO | ECHOE);

  tio.c_cc[VMIN] = 0;

  // Commit
  if(tcsetattr(aFd,TCSADRAIN,&tio)==0)
    return 1;
  
  tcflush(aFd, TCIOFLUSH);
  return 0;
}

FILE* SerialReader::openSerial(const char *aDevName, int aSpeed)
{
  FILE *dev;
	dev = fopen(aDevName,"w+");
  if(dev==NULL)
    return NULL;  // invalid device file

  // setup parameters
  if(setupSerial(fileno(dev),aSpeed))
  {
	  fflush(dev);
	  tcflush(fileno(dev),TCIOFLUSH);
	  return dev;
  }
    
  // fail
  fclose(dev);
  return NULL;
}

bool SerialReader::getPacket(char spacket[1024], unsigned char &packet_type, unsigned short &packet_crc)
{
	char stoken[1];
  char ssize[2];
  char stype[1];
  char scrc[2];

  unsigned short packet_size;

  int i;

  // get beginning (">*>")

  i = fread (stoken, sizeof(char), 1, dev);
  if (i == 0) return false;
  if (stoken[0] != '>') return false;

  i = fread (stoken, sizeof(char), 1, dev);
  if (i == 0) return false;
  if (stoken[0] != '*') return false;

  i = fread (stoken, sizeof(char), 1, dev);
  if (i == 0) return false;
  if (stoken[0] != '>') return false;

  //printf("\tBeginning string: ok\n");

  // get packet size
  
  i = fread(ssize, sizeof(char), 2, dev);
  if (i == 0) return false;
  memcpy(&packet_size, ssize, sizeof(packet_size));

  //printf("\tPacket size: %d\n", packet_size);

  // get packet type

  i = fread(stype, sizeof(char), 1, dev);
  if (i == 0) return false;
  memcpy(&packet_type, stype, sizeof(packet_type));

  //printf("\tPacket type: %d\n", packet_type);

  // get packet
  
  i = fread(spacket, sizeof(char), packet_size, dev);
  if (i == 0) return false;

  //printf("\tPacket string: ok\n");

  // get packet crc

  i = fread(scrc, sizeof(char), sizeof(scrc), dev);
  if (i == 0) return false;
  memcpy(&packet_crc, scrc, sizeof(packet_crc));

  //printf("\tPacket crc: %d\n", packet_crc);

  // get closing ("<#<")

  i = fread(stoken, sizeof(char), 1, dev);
  if (i == 0) return false;
  if (stoken[0] != '<') return false;

  i = fread(stoken, sizeof(char), 1, dev);
  if (i == 0) return false;
  if (stoken[0] != '#') return false;

  i = fread(stoken, sizeof(char), 1, dev);
  if (i == 0) return false;
  if (stoken[0] != '<') return false;

 // printf("\tClosing string: ok\n");

  return true;
}

bool SerialReader::getIMUCalcdata(IMU_CALCDATA& data)
{
  char cmd[16];

  char spacket[1024];
  unsigned char packet_type;
  unsigned short packet_crc;

  unsigned short packets = 0x0004;

	sprintf(cmd, ">*>p%c", packets);
  //printf("Writing CMD: %s\n", cmd);

  int i = fwrite(cmd,sizeof(char),sizeof(cmd),dev);
  if(i == 0)
  {
    ROS_ERROR("Unable to write into Serial Port");
    return false;
  }
  
  // try to read in packet

  bool result = false;
  while(true)
  {
    //printf("getPacket started\n"); 
    bool read_result = getPacket(spacket, packet_type, packet_crc);

    if (read_result)
    {
      // successful read
      //printf("getPacket successful: type = %d, crc = %d\n", packet_type, packet_crc);

      if (packet_type == PD_IMUCALCDATA)
      {
        //printf("Packet type is IMUCALCDATA\n");

        IMU_CALCDATA m_data;
        memcpy(&m_data, spacket, sizeof(m_data));

        // calculate checksum

        unsigned short checksum = crc16(&m_data, sizeof(m_data));

        // checkusm is ok, copy temp data to data

        if (checksum == packet_crc)
        {
          data = m_data;
          result = true;
          break;
        }
      }
      else
      {
        printf("Packet type is UNKNOWN\n");
        break;
      }
    }
    else
    {
      // failed read
      printf("getPacket failed\n"); 

      break;
    }   
  }

  return result;
}

unsigned short SerialReader::crc_update (unsigned short crc, unsigned char data)
{
  data ^= (crc & 0xff);
  data ^= data << 4;

  return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
         ^ ((unsigned short )data << 3));
}

unsigned short SerialReader::crc16(void* data, unsigned short cnt)
{
  unsigned short crc=0xff;
  unsigned char * ptr=(unsigned char *) data;
  int i;

  for (i=0;i<cnt;i++)
  {
    crc=crc_update(crc,*ptr);
    ptr++;
  }
  return crc;
}


