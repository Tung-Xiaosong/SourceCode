// Generated by gencpp from file yhs_msgs/PathIsValidRequest.msg
// DO NOT EDIT!


#ifndef YHS_MSGS_MESSAGE_PATHISVALIDREQUEST_H
#define YHS_MSGS_MESSAGE_PATHISVALIDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>

namespace yhs_msgs
{
template <class ContainerAllocator>
struct PathIsValidRequest_
{
  typedef PathIsValidRequest_<ContainerAllocator> Type;

  PathIsValidRequest_()
    : header()
    , checked_path()  {
    }
  PathIsValidRequest_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , checked_path(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::nav_msgs::Path_<ContainerAllocator>  _checked_path_type;
  _checked_path_type checked_path;





  typedef boost::shared_ptr< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> const> ConstPtr;

}; // struct PathIsValidRequest_

typedef ::yhs_msgs::PathIsValidRequest_<std::allocator<void> > PathIsValidRequest;

typedef boost::shared_ptr< ::yhs_msgs::PathIsValidRequest > PathIsValidRequestPtr;
typedef boost::shared_ptr< ::yhs_msgs::PathIsValidRequest const> PathIsValidRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace yhs_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'nav_msgs': ['/opt/ros/kinetic/share/nav_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'yhs_msgs': ['/home/yhs/ME/src/yhs_bringup/yhs_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9c24c55c9069b3f86b12fac17109193e";
  }

  static const char* value(const ::yhs_msgs::PathIsValidRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9c24c55c9069b3f8ULL;
  static const uint64_t static_value2 = 0x6b12fac17109193eULL;
};

template<class ContainerAllocator>
struct DataType< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "yhs_msgs/PathIsValidRequest";
  }

  static const char* value(const ::yhs_msgs::PathIsValidRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
\n\
nav_msgs/Path checked_path\n\
\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: nav_msgs/Path\n\
#An array of poses that represents a Path for a robot to follow\n\
Header header\n\
geometry_msgs/PoseStamped[] poses\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::yhs_msgs::PathIsValidRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.checked_path);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PathIsValidRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::yhs_msgs::PathIsValidRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::yhs_msgs::PathIsValidRequest_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "checked_path: ";
    s << std::endl;
    Printer< ::nav_msgs::Path_<ContainerAllocator> >::stream(s, indent + "  ", v.checked_path);
  }
};

} // namespace message_operations
} // namespace ros

#endif // YHS_MSGS_MESSAGE_PATHISVALIDREQUEST_H
