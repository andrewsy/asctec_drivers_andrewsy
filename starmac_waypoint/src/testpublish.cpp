#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "asctec_msgs/WaypointCommand.h"//////
//#include "asctec_msgs/WaypointData.h"////////
#include <sstream>
#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>

#include <ros/ros.h>

#include "std_msgs/Bool.h"

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "waypoint_publisher");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n;


  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("WAYCOMMAND", 1000);
  ros::Publisher waycommand_pub;
  ros::Publisher waypoint_pub;


  waycommand_pub = n.advertise<std_msgs::Bool> ("WAYCOMMAND", 1); //asctec_msgs::WaypointCommand
  waypoint_pub = n.advertise<std_msgs::Bool> ("WAYPOINT", 1);  //asctec_msgs::WaypointData
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

   

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}


