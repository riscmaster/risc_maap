/* Auto-generated by genmsg_cpp for file /home/ece/ros_ws/src/foo_msgs/msg/Risc_rois.msg */
#ifndef FOO_MSGS_MESSAGE_RISC_ROIS_H
#define FOO_MSGS_MESSAGE_RISC_ROIS_H
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

#include "foo_msgs/Risc_roi.h"
#include "foo_msgs/Risc_roi.h"

namespace foo_msgs
{
template <class ContainerAllocator>
struct Risc_rois_ {
  typedef Risc_rois_<ContainerAllocator> Type;

  Risc_rois_()
  : name()
  , landmarks()
  , quads()
  {
  }

  Risc_rois_(const ContainerAllocator& _alloc)
  : name(_alloc)
  , landmarks(_alloc)
  , quads(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef std::vector< ::foo_msgs::Risc_roi_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::foo_msgs::Risc_roi_<ContainerAllocator> >::other >  _landmarks_type;
  std::vector< ::foo_msgs::Risc_roi_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::foo_msgs::Risc_roi_<ContainerAllocator> >::other >  landmarks;

  typedef std::vector< ::foo_msgs::Risc_roi_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::foo_msgs::Risc_roi_<ContainerAllocator> >::other >  _quads_type;
  std::vector< ::foo_msgs::Risc_roi_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::foo_msgs::Risc_roi_<ContainerAllocator> >::other >  quads;


  typedef boost::shared_ptr< ::foo_msgs::Risc_rois_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::foo_msgs::Risc_rois_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Risc_rois
typedef  ::foo_msgs::Risc_rois_<std::allocator<void> > Risc_rois;

typedef boost::shared_ptr< ::foo_msgs::Risc_rois> Risc_roisPtr;
typedef boost::shared_ptr< ::foo_msgs::Risc_rois const> Risc_roisConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::foo_msgs::Risc_rois_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::foo_msgs::Risc_rois_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace foo_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::foo_msgs::Risc_rois_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::foo_msgs::Risc_rois_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::foo_msgs::Risc_rois_<ContainerAllocator> > {
  static const char* value() 
  {
    return "f66579dedd062ccc57f1aced22cbbd3a";
  }

  static const char* value(const  ::foo_msgs::Risc_rois_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xf66579dedd062cccULL;
  static const uint64_t static_value2 = 0x57f1aced22cbbd3aULL;
};

template<class ContainerAllocator>
struct DataType< ::foo_msgs::Risc_rois_<ContainerAllocator> > {
  static const char* value() 
  {
    return "foo_msgs/Risc_rois";
  }

  static const char* value(const  ::foo_msgs::Risc_rois_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::foo_msgs::Risc_rois_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
Risc_roi[] landmarks\n\
Risc_roi[] quads\n\
\n\
================================================================================\n\
MSG: foo_msgs/Risc_roi\n\
string name\n\
\n\
bool visible\n\
\n\
int32 x\n\
\n\
int32 y\n\
\n\
float32 width\n\
\n\
float32 height\n\
\n\
float64 angle\n\
\n\
";
  }

  static const char* value(const  ::foo_msgs::Risc_rois_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::foo_msgs::Risc_rois_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.landmarks);
    stream.next(m.quads);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Risc_rois_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::foo_msgs::Risc_rois_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::foo_msgs::Risc_rois_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "landmarks[]" << std::endl;
    for (size_t i = 0; i < v.landmarks.size(); ++i)
    {
      s << indent << "  landmarks[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::foo_msgs::Risc_roi_<ContainerAllocator> >::stream(s, indent + "    ", v.landmarks[i]);
    }
    s << indent << "quads[]" << std::endl;
    for (size_t i = 0; i < v.quads.size(); ++i)
    {
      s << indent << "  quads[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::foo_msgs::Risc_roi_<ContainerAllocator> >::stream(s, indent + "    ", v.quads[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // FOO_MSGS_MESSAGE_RISC_ROIS_H

