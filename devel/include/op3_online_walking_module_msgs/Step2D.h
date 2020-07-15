// Generated by gencpp from file op3_online_walking_module_msgs/Step2D.msg
// DO NOT EDIT!


#ifndef OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_STEP2D_H
#define OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_STEP2D_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>

namespace op3_online_walking_module_msgs
{
template <class ContainerAllocator>
struct Step2D_
{
  typedef Step2D_<ContainerAllocator> Type;

  Step2D_()
    : step2d()
    , moving_foot(0)  {
    }
  Step2D_(const ContainerAllocator& _alloc)
    : step2d(_alloc)
    , moving_foot(0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _step2d_type;
  _step2d_type step2d;

   typedef uint8_t _moving_foot_type;
  _moving_foot_type moving_foot;



  enum {
    LEFT_FOOT_SWING = 1u,
    RIGHT_FOOT_SWING = 2u,
    STANDING = 3u,
  };


  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> const> ConstPtr;

}; // struct Step2D_

typedef ::op3_online_walking_module_msgs::Step2D_<std::allocator<void> > Step2D;

typedef boost::shared_ptr< ::op3_online_walking_module_msgs::Step2D > Step2DPtr;
typedef boost::shared_ptr< ::op3_online_walking_module_msgs::Step2D const> Step2DConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace op3_online_walking_module_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'op3_online_walking_module_msgs': ['/home/robotis/christmann_ws/src/ROBOTIS-OP3-msgs/op3_online_walking_module_msgs/msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8b716dffcd181458918724c59549dd00";
  }

  static const char* value(const ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8b716dffcd181458ULL;
  static const uint64_t static_value2 = 0x918724c59549dd00ULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_online_walking_module_msgs/Step2D";
  }

  static const char* value(const ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#2D StepData\n\
\n\
geometry_msgs/Pose2D step2d   # step pose as relative offset to last leg\n\
\n\
\n\
# which leg to be used (left/right/no, see below)\n\
uint8 moving_foot   \n\
\n\
uint8 LEFT_FOOT_SWING  = 1 # Left foot constant\n\
uint8 RIGHT_FOOT_SWING = 2 # Right foot constant\n\
uint8 STANDING         = 3 # Standing constant\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.step2d);
      stream.next(m.moving_foot);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Step2D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::op3_online_walking_module_msgs::Step2D_<ContainerAllocator>& v)
  {
    s << indent << "step2d: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.step2d);
    s << indent << "moving_foot: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.moving_foot);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OP3_ONLINE_WALKING_MODULE_MSGS_MESSAGE_STEP2D_H
