/* Auto-generated by genmsg_cpp for file /home/ece/ros_ws/src/foo_msgs/msg/States.msg */
#ifndef FOO_MSGS_MESSAGE_STATES_H
#define FOO_MSGS_MESSAGE_STATES_H
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
struct States_ {
  typedef States_<ContainerAllocator> Type;

  States_()
  : name()
  , visible(false)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , u(0.0)
  , v(0.0)
  , w(0.0)
  , phi(0.0)
  , theta(0.0)
  , psi(0.0)
  , p(0.0)
  , q(0.0)
  , r(0.0)
  {
  }

  States_(const ContainerAllocator& _alloc)
  : name(_alloc)
  , visible(false)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  , u(0.0)
  , v(0.0)
  , w(0.0)
  , phi(0.0)
  , theta(0.0)
  , psi(0.0)
  , p(0.0)
  , q(0.0)
  , r(0.0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef uint8_t _visible_type;
  uint8_t visible;

  typedef double _x_type;
  double x;

  typedef double _y_type;
  double y;

  typedef double _z_type;
  double z;

  typedef double _u_type;
  double u;

  typedef double _v_type;
  double v;

  typedef double _w_type;
  double w;

  typedef double _phi_type;
  double phi;

  typedef double _theta_type;
  double theta;

  typedef double _psi_type;
  double psi;

  typedef double _p_type;
  double p;

  typedef double _q_type;
  double q;

  typedef double _r_type;
  double r;


  typedef boost::shared_ptr< ::foo_msgs::States_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::foo_msgs::States_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct States
typedef  ::foo_msgs::States_<std::allocator<void> > States;

typedef boost::shared_ptr< ::foo_msgs::States> StatesPtr;
typedef boost::shared_ptr< ::foo_msgs::States const> StatesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::foo_msgs::States_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::foo_msgs::States_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace foo_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::foo_msgs::States_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::foo_msgs::States_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::foo_msgs::States_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8271f59b5179794f309908101ca21e03";
  }

  static const char* value(const  ::foo_msgs::States_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8271f59b5179794fULL;
  static const uint64_t static_value2 = 0x309908101ca21e03ULL;
};

template<class ContainerAllocator>
struct DataType< ::foo_msgs::States_<ContainerAllocator> > {
  static const char* value() 
  {
    return "foo_msgs/States";
  }

  static const char* value(const  ::foo_msgs::States_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::foo_msgs::States_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
\n\
bool visible\n\
\n\
float64 x\n\
\n\
float64 y\n\
\n\
float64 z\n\
\n\
float64 u\n\
\n\
float64 v\n\
\n\
float64 w\n\
\n\
float64 phi\n\
\n\
float64 theta\n\
\n\
float64 psi\n\
\n\
float64 p\n\
\n\
float64 q\n\
\n\
float64 r\n\
\n\
";
  }

  static const char* value(const  ::foo_msgs::States_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::foo_msgs::States_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.visible);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
    stream.next(m.u);
    stream.next(m.v);
    stream.next(m.w);
    stream.next(m.phi);
    stream.next(m.theta);
    stream.next(m.psi);
    stream.next(m.p);
    stream.next(m.q);
    stream.next(m.r);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct States_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::foo_msgs::States_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::foo_msgs::States_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "visible: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.visible);
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<double>::stream(s, indent + "  ", v.z);
    s << indent << "u: ";
    Printer<double>::stream(s, indent + "  ", v.u);
    s << indent << "v: ";
    Printer<double>::stream(s, indent + "  ", v.v);
    s << indent << "w: ";
    Printer<double>::stream(s, indent + "  ", v.w);
    s << indent << "phi: ";
    Printer<double>::stream(s, indent + "  ", v.phi);
    s << indent << "theta: ";
    Printer<double>::stream(s, indent + "  ", v.theta);
    s << indent << "psi: ";
    Printer<double>::stream(s, indent + "  ", v.psi);
    s << indent << "p: ";
    Printer<double>::stream(s, indent + "  ", v.p);
    s << indent << "q: ";
    Printer<double>::stream(s, indent + "  ", v.q);
    s << indent << "r: ";
    Printer<double>::stream(s, indent + "  ", v.r);
  }
};


} // namespace message_operations
} // namespace ros

#endif // FOO_MSGS_MESSAGE_STATES_H
