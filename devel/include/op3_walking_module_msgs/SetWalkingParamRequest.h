// Generated by gencpp from file op3_walking_module_msgs/SetWalkingParamRequest.msg
// DO NOT EDIT!


#ifndef OP3_WALKING_MODULE_MSGS_MESSAGE_SETWALKINGPARAMREQUEST_H
#define OP3_WALKING_MODULE_MSGS_MESSAGE_SETWALKINGPARAMREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <op3_walking_module_msgs/WalkingParam.h>

namespace op3_walking_module_msgs
{
template <class ContainerAllocator>
struct SetWalkingParamRequest_
{
  typedef SetWalkingParamRequest_<ContainerAllocator> Type;

  SetWalkingParamRequest_()
    : parameters()  {
    }
  SetWalkingParamRequest_(const ContainerAllocator& _alloc)
    : parameters(_alloc)  {
  (void)_alloc;
    }



   typedef  ::op3_walking_module_msgs::WalkingParam_<ContainerAllocator>  _parameters_type;
  _parameters_type parameters;





  typedef boost::shared_ptr< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetWalkingParamRequest_

typedef ::op3_walking_module_msgs::SetWalkingParamRequest_<std::allocator<void> > SetWalkingParamRequest;

typedef boost::shared_ptr< ::op3_walking_module_msgs::SetWalkingParamRequest > SetWalkingParamRequestPtr;
typedef boost::shared_ptr< ::op3_walking_module_msgs::SetWalkingParamRequest const> SetWalkingParamRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace op3_walking_module_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'op3_walking_module_msgs': ['/home/robotis/christmann_ws/src/ROBOTIS-OP3-msgs/op3_walking_module_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "351c78d980e15976e0c1bcf480ef041c";
  }

  static const char* value(const ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x351c78d980e15976ULL;
  static const uint64_t static_value2 = 0xe0c1bcf480ef041cULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_walking_module_msgs/SetWalkingParamRequest";
  }

  static const char* value(const ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "WalkingParam    parameters\n\
\n\
================================================================================\n\
MSG: op3_walking_module_msgs/WalkingParam\n\
####### walking init pose #######\n\
float32 init_x_offset\n\
float32 init_y_offset\n\
float32 init_z_offset\n\
float32 init_roll_offset\n\
float32 init_pitch_offset\n\
float32 init_yaw_offset\n\
\n\
####### time parameter #####\n\
float32 period_time\n\
float32 dsp_ratio\n\
float32 step_fb_ratio\n\
\n\
########## walking parameter ########\n\
float32 x_move_amplitude\n\
float32 y_move_amplitude\n\
float32 z_move_amplitude\n\
float32 angle_move_amplitude\n\
bool move_aim_on\n\
\n\
########## balance parameter ##########\n\
bool balance_enable\n\
float32 balance_hip_roll_gain\n\
float32 balance_knee_gain\n\
float32 balance_ankle_roll_gain\n\
float32 balance_ankle_pitch_gain\n\
float32 y_swap_amplitude\n\
float32 z_swap_amplitude\n\
float32 arm_swing_gain\n\
float32 pelvis_offset\n\
float32 hip_pitch_offset\n\
\n\
########## gain parameter ##########\n\
int32 p_gain\n\
int32 i_gain\n\
int32 d_gain\n\
";
  }

  static const char* value(const ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.parameters);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetWalkingParamRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::op3_walking_module_msgs::SetWalkingParamRequest_<ContainerAllocator>& v)
  {
    s << indent << "parameters: ";
    s << std::endl;
    Printer< ::op3_walking_module_msgs::WalkingParam_<ContainerAllocator> >::stream(s, indent + "  ", v.parameters);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OP3_WALKING_MODULE_MSGS_MESSAGE_SETWALKINGPARAMREQUEST_H
