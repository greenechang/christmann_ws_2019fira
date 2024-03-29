// Generated by gencpp from file op3_camera_setting_tool/GetParametersRequest.msg
// DO NOT EDIT!


#ifndef OP3_CAMERA_SETTING_TOOL_MESSAGE_GETPARAMETERSREQUEST_H
#define OP3_CAMERA_SETTING_TOOL_MESSAGE_GETPARAMETERSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace op3_camera_setting_tool
{
template <class ContainerAllocator>
struct GetParametersRequest_
{
  typedef GetParametersRequest_<ContainerAllocator> Type;

  GetParametersRequest_()
    {
    }
  GetParametersRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetParametersRequest_

typedef ::op3_camera_setting_tool::GetParametersRequest_<std::allocator<void> > GetParametersRequest;

typedef boost::shared_ptr< ::op3_camera_setting_tool::GetParametersRequest > GetParametersRequestPtr;
typedef boost::shared_ptr< ::op3_camera_setting_tool::GetParametersRequest const> GetParametersRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace op3_camera_setting_tool

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'op3_camera_setting_tool': ['/home/robotis/christmann_ws/src/ROBOTIS-OP3-Tools/op3_camera_setting_tool/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "op3_camera_setting_tool/GetParametersRequest";
  }

  static const char* value(const ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetParametersRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::op3_camera_setting_tool::GetParametersRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // OP3_CAMERA_SETTING_TOOL_MESSAGE_GETPARAMETERSREQUEST_H
