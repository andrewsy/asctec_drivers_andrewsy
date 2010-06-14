#include <ros/ros.h>
#include "serial_reader/serial_reader_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "serialReader");

  SerialReaderNode serialReaderNode;

  ros::spin();

	return 0;
}
