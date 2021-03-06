/* Auto-generated by genmsg_cpp for file /home/andrewsy/ros/starmac-ros-pkg/starmac_flyer/flyer_controller/msg/Autosequence.msg */
#ifndef FLYER_CONTROLLER_MESSAGE_AUTOSEQUENCE_H
#define FLYER_CONTROLLER_MESSAGE_AUTOSEQUENCE_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "flyer_controller/AutosequencePoint.h"

namespace flyer_controller
{
template <class ContainerAllocator>
struct Autosequence_ : public ros::Message
{
  typedef Autosequence_<ContainerAllocator> Type;

  Autosequence_()
  : name()
  , num_points(0)
  , points()
  {
  }

  Autosequence_(const ContainerAllocator& _alloc)
  : name(_alloc)
  , num_points(0)
  , points(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef uint32_t _num_points_type;
  uint32_t num_points;

  typedef std::vector< ::flyer_controller::AutosequencePoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::flyer_controller::AutosequencePoint_<ContainerAllocator> >::other >  _points_type;
  std::vector< ::flyer_controller::AutosequencePoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::flyer_controller::AutosequencePoint_<ContainerAllocator> >::other >  points;


  ROS_DEPRECATED uint32_t get_points_size() const { return (uint32_t)points.size(); }
  ROS_DEPRECATED void set_points_size(uint32_t size) { points.resize((size_t)size); }
  ROS_DEPRECATED void get_points_vec(std::vector< ::flyer_controller::AutosequencePoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::flyer_controller::AutosequencePoint_<ContainerAllocator> >::other > & vec) const { vec = this->points; }
  ROS_DEPRECATED void set_points_vec(const std::vector< ::flyer_controller::AutosequencePoint_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::flyer_controller::AutosequencePoint_<ContainerAllocator> >::other > & vec) { this->points = vec; }
private:
  static const char* __s_getDataType_() { return "flyer_controller/Autosequence"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "2fc3d91e94190d44de5f18ae9fa72ffd"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "string name\n\
uint32 num_points\n\
flyer_controller/AutosequencePoint[] points\n\
================================================================================\n\
MSG: flyer_controller/AutosequencePoint\n\
flyer_controller/HoverPoint hover_point\n\
bool pause\n\
================================================================================\n\
MSG: flyer_controller/HoverPoint\n\
string name\n\
float64 x # [m] (North)\n\
float64 y # [m] (East)\n\
float64 alt # [m]\n\
float64 yaw # [deg]\n\
float64 vx # [m/s]\n\
float64 vy # [m/s]\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, name);
    ros::serialization::serialize(stream, num_points);
    ros::serialization::serialize(stream, points);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, name);
    ros::serialization::deserialize(stream, num_points);
    ros::serialization::deserialize(stream, points);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(name);
    size += ros::serialization::serializationLength(num_points);
    size += ros::serialization::serializationLength(points);
    return size;
  }

  typedef boost::shared_ptr< ::flyer_controller::Autosequence_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::flyer_controller::Autosequence_<ContainerAllocator>  const> ConstPtr;
}; // struct Autosequence
typedef  ::flyer_controller::Autosequence_<std::allocator<void> > Autosequence;

typedef boost::shared_ptr< ::flyer_controller::Autosequence> AutosequencePtr;
typedef boost::shared_ptr< ::flyer_controller::Autosequence const> AutosequenceConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::flyer_controller::Autosequence_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::flyer_controller::Autosequence_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace flyer_controller

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::flyer_controller::Autosequence_<ContainerAllocator> > {
  static const char* value() 
  {
    return "2fc3d91e94190d44de5f18ae9fa72ffd";
  }

  static const char* value(const  ::flyer_controller::Autosequence_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x2fc3d91e94190d44ULL;
  static const uint64_t static_value2 = 0xde5f18ae9fa72ffdULL;
};

template<class ContainerAllocator>
struct DataType< ::flyer_controller::Autosequence_<ContainerAllocator> > {
  static const char* value() 
  {
    return "flyer_controller/Autosequence";
  }

  static const char* value(const  ::flyer_controller::Autosequence_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::flyer_controller::Autosequence_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
uint32 num_points\n\
flyer_controller/AutosequencePoint[] points\n\
================================================================================\n\
MSG: flyer_controller/AutosequencePoint\n\
flyer_controller/HoverPoint hover_point\n\
bool pause\n\
================================================================================\n\
MSG: flyer_controller/HoverPoint\n\
string name\n\
float64 x # [m] (North)\n\
float64 y # [m] (East)\n\
float64 alt # [m]\n\
float64 yaw # [deg]\n\
float64 vx # [m/s]\n\
float64 vy # [m/s]\n\
";
  }

  static const char* value(const  ::flyer_controller::Autosequence_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::flyer_controller::Autosequence_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.num_points);
    stream.next(m.points);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Autosequence_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::flyer_controller::Autosequence_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::flyer_controller::Autosequence_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "num_points: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.num_points);
    s << indent << "points[]" << std::endl;
    for (size_t i = 0; i < v.points.size(); ++i)
    {
      s << indent << "  points[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::flyer_controller::AutosequencePoint_<ContainerAllocator> >::stream(s, indent + "    ", v.points[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // FLYER_CONTROLLER_MESSAGE_AUTOSEQUENCE_H

