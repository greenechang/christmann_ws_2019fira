// Generated by gencpp from file robotis_controller_msgs/SyncWriteItem.msg
// DO NOT EDIT!


#ifndef ROBOTIS_CONTROLLER_MSGS_MESSAGE_SYNCWRITEITEM_H
#define ROBOTIS_CONTROLLER_MSGS_MESSAGE_SYNCWRITEITEM_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotis_controller_msgs
{
template <class ContainerAllocator>
struct SyncWriteItem_
{
  typedef SyncWriteItem_<ContainerAllocator> Type;

  SyncWriteItem_()
    : item_name()
    , joint_name()
    , value()  {
    }
  SyncWriteItem_(const ContainerAllocator& _alloc)
    : item_name(_alloc)
    , joint_name(_alloc)
    , value(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _item_name_type;
  _item_name_type item_name;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _joint_name_type;
  _joint_name_type joint_name;

   typedef std::vector<uint32_t, typename ContainerAllocator::template rebind<uint32_t>::other >  _value_type;
  _value_type value;





  typedef boost::shared_ptr< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> const> ConstPtr;

}; // struct SyncWriteItem_

typedef ::robotis_controller_msgs::SyncWriteItem_<std::allocator<void> > SyncWriteItem;

typedef boost::shared_ptr< ::robotis_controller_msgs::SyncWriteItem > SyncWriteItemPtr;
typedef boost::shared_ptr< ::robotis_controller_msgs::SyncWriteItem const> SyncWriteItemConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace robotis_controller_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'robotis_controller_msgs': ['/home/robotis/christmann_ws/src/ROBOTIS-Framework-msgs/robotis_controller_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f558e04f04dbcc25ce64aa1f45f3dbdd";
  }

  static const char* value(const ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf558e04f04dbcc25ULL;
  static const uint64_t static_value2 = 0xce64aa1f45f3dbddULL;
};

template<class ContainerAllocator>
struct DataType< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotis_controller_msgs/SyncWriteItem";
  }

  static const char* value(const ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string item_name\n\
string[] joint_name\n\
uint32[] value\n\
";
  }

  static const char* value(const ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.item_name);
      stream.next(m.joint_name);
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SyncWriteItem_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotis_controller_msgs::SyncWriteItem_<ContainerAllocator>& v)
  {
    s << indent << "item_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.item_name);
    s << indent << "joint_name[]" << std::endl;
    for (size_t i = 0; i < v.joint_name.size(); ++i)
    {
      s << indent << "  joint_name[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_name[i]);
    }
    s << indent << "value[]" << std::endl;
    for (size_t i = 0; i < v.value.size(); ++i)
    {
      s << indent << "  value[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.value[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTIS_CONTROLLER_MSGS_MESSAGE_SYNCWRITEITEM_H
