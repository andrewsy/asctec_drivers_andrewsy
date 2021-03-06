/* Auto-generated by genmsg_cpp for file /home/andrewsy/ros/starmac-ros-pkg/starmac_flyer/flyer_controller/msg/control_mode_autosequence_info.msg */
#ifndef FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_AUTOSEQUENCE_INFO_H
#define FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_AUTOSEQUENCE_INFO_H
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
struct control_mode_autosequence_info_ : public ros::Message
{
  typedef control_mode_autosequence_info_<ContainerAllocator> Type;

  control_mode_autosequence_info_()
  : header()
  , status(0)
  , current_autosequence()
  , current_point(0)
  , defined_autosequences()
  {
  }

  control_mode_autosequence_info_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , status(0)
  , current_autosequence(_alloc)
  , current_point(0)
  , defined_autosequences(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int8_t _status_type;
  int8_t status;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _current_autosequence_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  current_autosequence;

  typedef uint32_t _current_point_type;
  uint32_t current_point;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _defined_autosequences_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  defined_autosequences;

  enum { WAITING_PROCEED = 0 };
  enum { MOVING = 1 };
  enum { PAUSED = 2 };
  enum { IDLE = 3 };

  ROS_DEPRECATED uint32_t get_defined_autosequences_size() const { return (uint32_t)defined_autosequences.size(); }
  ROS_DEPRECATED void set_defined_autosequences_size(uint32_t size) { defined_autosequences.resize((size_t)size); }
  ROS_DEPRECATED void get_defined_autosequences_vec(std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other > & vec) const { vec = this->defined_autosequences; }
  ROS_DEPRECATED void set_defined_autosequences_vec(const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other > & vec) { this->defined_autosequences = vec; }
private:
  static const char* __s_getDataType_() { return "flyer_controller/control_mode_autosequence_info"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "23f909b9602b395bbe12d106602f3d6d"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 WAITING_PROCEED = 0\n\
int8 MOVING = 1\n\
int8 PAUSED = 2\n\
int8 IDLE = 3\n\
\n\
Header header\n\
int8 status\n\
string current_autosequence\n\
uint32 current_point\n\
string[] defined_autosequences\n\
\n\
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
    ros::serialization::serialize(stream, status);
    ros::serialization::serialize(stream, current_autosequence);
    ros::serialization::serialize(stream, current_point);
    ros::serialization::serialize(stream, defined_autosequences);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, status);
    ros::serialization::deserialize(stream, current_autosequence);
    ros::serialization::deserialize(stream, current_point);
    ros::serialization::deserialize(stream, defined_autosequences);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(status);
    size += ros::serialization::serializationLength(current_autosequence);
    size += ros::serialization::serializationLength(current_point);
    size += ros::serialization::serializationLength(defined_autosequences);
    return size;
  }

  typedef boost::shared_ptr< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator>  const> ConstPtr;
}; // struct control_mode_autosequence_info
typedef  ::flyer_controller::control_mode_autosequence_info_<std::allocator<void> > control_mode_autosequence_info;

typedef boost::shared_ptr< ::flyer_controller::control_mode_autosequence_info> control_mode_autosequence_infoPtr;
typedef boost::shared_ptr< ::flyer_controller::control_mode_autosequence_info const> control_mode_autosequence_infoConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace flyer_controller

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "23f909b9602b395bbe12d106602f3d6d";
  }

  static const char* value(const  ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x23f909b9602b395bULL;
  static const uint64_t static_value2 = 0xbe12d106602f3d6dULL;
};

template<class ContainerAllocator>
struct DataType< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "flyer_controller/control_mode_autosequence_info";
  }

  static const char* value(const  ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 WAITING_PROCEED = 0\n\
int8 MOVING = 1\n\
int8 PAUSED = 2\n\
int8 IDLE = 3\n\
\n\
Header header\n\
int8 status\n\
string current_autosequence\n\
uint32 current_point\n\
string[] defined_autosequences\n\
\n\
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

  static const char* value(const  ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.status);
    stream.next(m.current_autosequence);
    stream.next(m.current_point);
    stream.next(m.defined_autosequences);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct control_mode_autosequence_info_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::flyer_controller::control_mode_autosequence_info_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "status: ";
    Printer<int8_t>::stream(s, indent + "  ", v.status);
    s << indent << "current_autosequence: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.current_autosequence);
    s << indent << "current_point: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.current_point);
    s << indent << "defined_autosequences[]" << std::endl;
    for (size_t i = 0; i < v.defined_autosequences.size(); ++i)
    {
      s << indent << "  defined_autosequences[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.defined_autosequences[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_AUTOSEQUENCE_INFO_H

