/* Auto-generated by genmsg_cpp for file /home/andrewsy/ros/starmac-ros-pkg/starmac_flyer/flyer_controller/msg/control_mode_cmd.msg */
#ifndef FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_CMD_H
#define FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_CMD_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"


namespace flyer_controller
{
template <class ContainerAllocator>
struct control_mode_cmd_ : public ros::Message
{
  typedef control_mode_cmd_<ContainerAllocator> Type;

  control_mode_cmd_()
  : cmd()
  {
  }

  control_mode_cmd_(const ContainerAllocator& _alloc)
  : cmd(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cmd_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  cmd;


private:
  static const char* __s_getDataType_() { return "flyer_controller/control_mode_cmd"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "43a54fa49066cddcf148717d9d4a6353"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "string cmd\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, cmd);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, cmd);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(cmd);
    return size;
  }

  typedef boost::shared_ptr< ::flyer_controller::control_mode_cmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::flyer_controller::control_mode_cmd_<ContainerAllocator>  const> ConstPtr;
}; // struct control_mode_cmd
typedef  ::flyer_controller::control_mode_cmd_<std::allocator<void> > control_mode_cmd;

typedef boost::shared_ptr< ::flyer_controller::control_mode_cmd> control_mode_cmdPtr;
typedef boost::shared_ptr< ::flyer_controller::control_mode_cmd const> control_mode_cmdConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::flyer_controller::control_mode_cmd_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::flyer_controller::control_mode_cmd_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace flyer_controller

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::flyer_controller::control_mode_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "43a54fa49066cddcf148717d9d4a6353";
  }

  static const char* value(const  ::flyer_controller::control_mode_cmd_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x43a54fa49066cddcULL;
  static const uint64_t static_value2 = 0xf148717d9d4a6353ULL;
};

template<class ContainerAllocator>
struct DataType< ::flyer_controller::control_mode_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "flyer_controller/control_mode_cmd";
  }

  static const char* value(const  ::flyer_controller::control_mode_cmd_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::flyer_controller::control_mode_cmd_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string cmd\n\
";
  }

  static const char* value(const  ::flyer_controller::control_mode_cmd_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::flyer_controller::control_mode_cmd_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.cmd);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct control_mode_cmd_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::flyer_controller::control_mode_cmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::flyer_controller::control_mode_cmd_<ContainerAllocator> & v) 
  {
    s << indent << "cmd: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.cmd);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FLYER_CONTROLLER_MESSAGE_CONTROL_MODE_CMD_H

