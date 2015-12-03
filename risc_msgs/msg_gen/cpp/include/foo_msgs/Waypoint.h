/* Auto-generated by genmsg_cpp for file /home/ece/ros_ws/src/foo_msgs/msg/Waypoint.msg */
#ifndef FOO_MSGS_MESSAGE_WAYPOINT_H
#define FOO_MSGS_MESSAGE_WAYPOINT_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace foo_msgs
{
template <class ContainerAllocator>
struct Waypoint_ {
  typedef Waypoint_<ContainerAllocator> Type;

  Waypoint_()
  : name()
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , heading(0.0)
  {
  }

  Waypoint_(const ContainerAllocator& _alloc)
  : name(_alloc)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , heading(0.0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _z_type;
  double z;

  typedef double _heading_type;
  double heading;


  typedef boost::shared_ptr< ::foo_msgs::Waypoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::foo_msgs::Waypoint_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Waypoint
typedef  ::foo_msgs::Waypoint_<std::allocator<void> > Waypoint;

typedef boost::shared_ptr< ::foo_msgs::Waypoint> WaypointPtr;
typedef boost::shared_ptr< ::foo_msgs::Waypoint const> WaypointConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::foo_msgs::Waypoint_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::foo_msgs::Waypoint_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace foo_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::foo_msgs::Waypoint_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::foo_msgs::Waypoint_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::foo_msgs::Waypoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "c0027f92b0acc03e6886dd3a0b948e52";
  }

  static const char* value(const  ::foo_msgs::Waypoint_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xc0027f92b0acc03eULL;
  static const uint64_t static_value2 = 0x6886dd3a0b948e52ULL;
};

template<class ContainerAllocator>
struct DataType< ::foo_msgs::Waypoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "foo_msgs/Waypoint";
  }

  static const char* value(const  ::foo_msgs::Waypoint_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::foo_msgs::Waypoint_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 heading\n\
\n\
";
  }

  static const char* value(const  ::foo_msgs::Waypoint_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::foo_msgs::Waypoint_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.heading);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Waypoint_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::foo_msgs::Waypoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::foo_msgs::Waypoint_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "heading: ";
    Printer<double>::stream(s, indent + "  ", v.heading);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FOO_MSGS_MESSAGE_WAYPOINT_H
