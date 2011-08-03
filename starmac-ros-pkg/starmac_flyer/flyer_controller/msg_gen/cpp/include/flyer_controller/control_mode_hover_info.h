/* Auto-generated by genmsg_cpp for file /home/andrewsy/ros/starmac-ros-pkg/starmac_flyer/flyer_controller/msg/control_mode_hover_info.msg */
#ifndef FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_HOVER_INFO_H
#define FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_HOVER_INFO_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "std_msgs/Header.h"

namespace flyer_controller
{
template <class ContainerAllocator>
struct control_mode_hover_info_ : public ros::Message
{
  typedef control_mode_hover_info_<ContainerAllocator> Type;

  control_mode_hover_info_()
  : header()
  , hover_point()
  , north_cmd(0.0)
  , east_cmd(0.0)
  , north_vel_cmd(0.0)
  , east_vel_cmd(0.0)
  , yaw_cmd(0.0)
  , alt_override(0.0)
  , north_err(0.0)
  , east_err(0.0)
  , north_vel_err(0.0)
  , east_vel_err(0.0)
  , yaw_err(0.0)
  {
  }

  control_mode_hover_info_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , hover_point(_alloc)
  , north_cmd(0.0)
  , east_cmd(0.0)
  , north_vel_cmd(0.0)
  , east_vel_cmd(0.0)
  , yaw_cmd(0.0)
  , alt_override(0.0)
  , north_err(0.0)
  , east_err(0.0)
  , north_vel_err(0.0)
  , east_vel_err(0.0)
  , yaw_err(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _hover_point_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  hover_point;

  typedef double _north_cmd_type;
  double north_cmd;

  typedef double _east_cmd_type;
  double east_cmd;

  typedef double _north_vel_cmd_type;
  double north_vel_cmd;

  typedef double _east_vel_cmd_type;
  double east_vel_cmd;

  typedef double _yaw_cmd_type;
  double yaw_cmd;

  typedef double _alt_override_type;
  double alt_override;

  typedef double _north_err_type;
  double north_err;

  typedef double _east_err_type;
  double east_err;

  typedef double _north_vel_err_type;
  double north_vel_err;

  typedef double _east_vel_err_type;
  double east_vel_err;

  typedef double _yaw_err_type;
  double yaw_err;


private:
  static const char* __s_getDataType_() { return "flyer_controller/control_mode_hover_info"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "67aa3c03432b61b4cc2a5316a0d1458e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "Header header\n\
string hover_point\n\
\n\
float64 north_cmd\n\
float64 east_cmd\n\
float64 north_vel_cmd\n\
float64 east_vel_cmd\n\
float64 yaw_cmd\n\
\n\
float64 alt_override\n\
\n\
float64 north_err\n\
float64 east_err\n\
float64 north_vel_err\n\
float64 east_vel_err\n\
float64 yaw_err\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, hover_point);
    ros::serialization::serialize(stream, north_cmd);
    ros::serialization::serialize(stream, east_cmd);
    ros::serialization::serialize(stream, north_vel_cmd);
    ros::serialization::serialize(stream, east_vel_cmd);
    ros::serialization::serialize(stream, yaw_cmd);
    ros::serialization::serialize(stream, alt_override);
    ros::serialization::serialize(stream, north_err);
    ros::serialization::serialize(stream, east_err);
    ros::serialization::serialize(stream, north_vel_err);
    ros::serialization::serialize(stream, east_vel_err);
    ros::serialization::serialize(stream, yaw_err);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, hover_point);
    ros::serialization::deserialize(stream, north_cmd);
    ros::serialization::deserialize(stream, east_cmd);
    ros::serialization::deserialize(stream, north_vel_cmd);
    ros::serialization::deserialize(stream, east_vel_cmd);
    ros::serialization::deserialize(stream, yaw_cmd);
    ros::serialization::deserialize(stream, alt_override);
    ros::serialization::deserialize(stream, north_err);
    ros::serialization::deserialize(stream, east_err);
    ros::serialization::deserialize(stream, north_vel_err);
    ros::serialization::deserialize(stream, east_vel_err);
    ros::serialization::deserialize(stream, yaw_err);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(hover_point);
    size += ros::serialization::serializationLength(north_cmd);
    size += ros::serialization::serializationLength(east_cmd);
    size += ros::serialization::serializationLength(north_vel_cmd);
    size += ros::serialization::serializationLength(east_vel_cmd);
    size += ros::serialization::serializationLength(yaw_cmd);
    size += ros::serialization::serializationLength(alt_override);
    size += ros::serialization::serializationLength(north_err);
    size += ros::serialization::serializationLength(east_err);
    size += ros::serialization::serializationLength(north_vel_err);
    size += ros::serialization::serializationLength(east_vel_err);
    size += ros::serialization::serializationLength(yaw_err);
    return size;
  }

  typedef boost::shared_ptr< ::flyer_controller::control_mode_hover_info_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::flyer_controller::control_mode_hover_info_<ContainerAllocator>  const> ConstPtr;
}; // struct control_mode_hover_info
typedef  ::flyer_controller::control_mode_hover_info_<std::allocator<void> > control_mode_hover_info;

typedef boost::shared_ptr< ::flyer_controller::control_mode_hover_info> control_mode_hover_infoPtr;
typedef boost::shared_ptr< ::flyer_controller::control_mode_hover_info const> control_mode_hover_infoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::flyer_controller::control_mode_hover_info_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::flyer_controller::control_mode_hover_info_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace flyer_controller

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::flyer_controller::control_mode_hover_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "67aa3c03432b61b4cc2a5316a0d1458e";
  }

  static const char* value(const  ::flyer_controller::control_mode_hover_info_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x67aa3c03432b61b4ULL;
  static const uint64_t static_value2 = 0xcc2a5316a0d1458eULL;
};

template<class ContainerAllocator>
struct DataType< ::flyer_controller::control_mode_hover_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "flyer_controller/control_mode_hover_info";
  }

  static const char* value(const  ::flyer_controller::control_mode_hover_info_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::flyer_controller::control_mode_hover_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
string hover_point\n\
\n\
float64 north_cmd\n\
float64 east_cmd\n\
float64 north_vel_cmd\n\
float64 east_vel_cmd\n\
float64 yaw_cmd\n\
\n\
float64 alt_override\n\
\n\
float64 north_err\n\
float64 east_err\n\
float64 north_vel_err\n\
float64 east_vel_err\n\
float64 yaw_err\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::flyer_controller::control_mode_hover_info_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::flyer_controller::control_mode_hover_info_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::flyer_controller::control_mode_hover_info_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::flyer_controller::control_mode_hover_info_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.hover_point);
    stream.next(m.north_cmd);
    stream.next(m.east_cmd);
    stream.next(m.north_vel_cmd);
    stream.next(m.east_vel_cmd);
    stream.next(m.yaw_cmd);
    stream.next(m.alt_override);
    stream.next(m.north_err);
    stream.next(m.east_err);
    stream.next(m.north_vel_err);
    stream.next(m.east_vel_err);
    stream.next(m.yaw_err);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct control_mode_hover_info_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::flyer_controller::control_mode_hover_info_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::flyer_controller::control_mode_hover_info_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "hover_point: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.hover_point);
    s << indent << "north_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.north_cmd);
    s << indent << "east_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.east_cmd);
    s << indent << "north_vel_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.north_vel_cmd);
    s << indent << "east_vel_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.east_vel_cmd);
    s << indent << "yaw_cmd: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_cmd);
    s << indent << "alt_override: ";
    Printer<double>::stream(s, indent + "  ", v.alt_override);
    s << indent << "north_err: ";
    Printer<double>::stream(s, indent + "  ", v.north_err);
    s << indent << "east_err: ";
    Printer<double>::stream(s, indent + "  ", v.east_err);
    s << indent << "north_vel_err: ";
    Printer<double>::stream(s, indent + "  ", v.north_vel_err);
    s << indent << "east_vel_err: ";
    Printer<double>::stream(s, indent + "  ", v.east_vel_err);
    s << indent << "yaw_err: ";
    Printer<double>::stream(s, indent + "  ", v.yaw_err);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_HOVER_INFO_H
