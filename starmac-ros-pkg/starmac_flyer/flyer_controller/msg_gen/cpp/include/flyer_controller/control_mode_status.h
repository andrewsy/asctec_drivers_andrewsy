/* Auto-generated by genmsg_cpp for file /home/andrewsy/ros/starmac-ros-pkg/starmac_flyer/flyer_controller/msg/control_mode_status.msg */
#ifndef FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_STATUS_H
#define FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_STATUS_H
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
struct control_mode_status_ : public ros::Message
{
  typedef control_mode_status_<ContainerAllocator> Type;

  control_mode_status_()
  : header()
  , state(0)
  , ready(false)
  , info()
  {
  }

  control_mode_status_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , state(0)
  , ready(false)
  , info(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int8_t _state_type;
  int8_t state;

  typedef uint8_t _ready_type;
  uint8_t ready;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _info_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  info;

  enum { STATE_ERROR = 0 };
  enum { STATE_OFF = 1 };
  enum { STATE_IDLE = 2 };
  enum { STATE_STANDBY = 3 };
  enum { STATE_ACTIVE = 4 };

private:
  static const char* __s_getDataType_() { return "flyer_controller/control_mode_status"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "69f822cfeb8c98af2c7a1634de76aa9f"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 STATE_ERROR=0\n\
int8 STATE_OFF=1\n\
int8 STATE_IDLE=2\n\
int8 STATE_STANDBY=3\n\
int8 STATE_ACTIVE=4\n\
\n\
Header header\n\
int8 state\n\
bool ready # ready to transition to ACTIVE?\n\
string info\n\
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
    ros::serialization::serialize(stream, state);
    ros::serialization::serialize(stream, ready);
    ros::serialization::serialize(stream, info);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, state);
    ros::serialization::deserialize(stream, ready);
    ros::serialization::deserialize(stream, info);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(state);
    size += ros::serialization::serializationLength(ready);
    size += ros::serialization::serializationLength(info);
    return size;
  }

  typedef boost::shared_ptr< ::flyer_controller::control_mode_status_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::flyer_controller::control_mode_status_<ContainerAllocator>  const> ConstPtr;
}; // struct control_mode_status
typedef  ::flyer_controller::control_mode_status_<std::allocator<void> > control_mode_status;

typedef boost::shared_ptr< ::flyer_controller::control_mode_status> control_mode_statusPtr;
typedef boost::shared_ptr< ::flyer_controller::control_mode_status const> control_mode_statusConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::flyer_controller::control_mode_status_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::flyer_controller::control_mode_status_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace flyer_controller

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::flyer_controller::control_mode_status_<ContainerAllocator> > {
  static const char* value() 
  {
    return "69f822cfeb8c98af2c7a1634de76aa9f";
  }

  static const char* value(const  ::flyer_controller::control_mode_status_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x69f822cfeb8c98afULL;
  static const uint64_t static_value2 = 0x2c7a1634de76aa9fULL;
};

template<class ContainerAllocator>
struct DataType< ::flyer_controller::control_mode_status_<ContainerAllocator> > {
  static const char* value() 
  {
    return "flyer_controller/control_mode_status";
  }

  static const char* value(const  ::flyer_controller::control_mode_status_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::flyer_controller::control_mode_status_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 STATE_ERROR=0\n\
int8 STATE_OFF=1\n\
int8 STATE_IDLE=2\n\
int8 STATE_STANDBY=3\n\
int8 STATE_ACTIVE=4\n\
\n\
Header header\n\
int8 state\n\
bool ready # ready to transition to ACTIVE?\n\
string info\n\
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

  static const char* value(const  ::flyer_controller::control_mode_status_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::flyer_controller::control_mode_status_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::flyer_controller::control_mode_status_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::flyer_controller::control_mode_status_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.state);
    stream.next(m.ready);
    stream.next(m.info);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct control_mode_status_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::flyer_controller::control_mode_status_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::flyer_controller::control_mode_status_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "state: ";
    Printer<int8_t>::stream(s, indent + "  ", v.state);
    s << indent << "ready: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ready);
    s << indent << "info: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.info);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_STATUS_H
