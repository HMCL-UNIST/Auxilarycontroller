// Generated by gencpp from file autoware_msgs/ValueSet.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MSGS_MESSAGE_VALUESET_H
#define AUTOWARE_MSGS_MESSAGE_VALUESET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace autoware_msgs
{
template <class ContainerAllocator>
struct ValueSet_
{
  typedef ValueSet_<ContainerAllocator> Type;

  ValueSet_()
    : center(0)
    , range(0)  {
    }
  ValueSet_(const ContainerAllocator& _alloc)
    : center(0)
    , range(0)  {
  (void)_alloc;
    }



   typedef int32_t _center_type;
  _center_type center;

   typedef int32_t _range_type;
  _range_type range;





  typedef boost::shared_ptr< ::autoware_msgs::ValueSet_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_msgs::ValueSet_<ContainerAllocator> const> ConstPtr;

}; // struct ValueSet_

typedef ::autoware_msgs::ValueSet_<std::allocator<void> > ValueSet;

typedef boost::shared_ptr< ::autoware_msgs::ValueSet > ValueSetPtr;
typedef boost::shared_ptr< ::autoware_msgs::ValueSet const> ValueSetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_msgs::ValueSet_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_msgs::ValueSet_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_msgs::ValueSet_<ContainerAllocator1> & lhs, const ::autoware_msgs::ValueSet_<ContainerAllocator2> & rhs)
{
  return lhs.center == rhs.center &&
    lhs.range == rhs.range;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_msgs::ValueSet_<ContainerAllocator1> & lhs, const ::autoware_msgs::ValueSet_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::ValueSet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::ValueSet_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::ValueSet_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::ValueSet_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::ValueSet_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::ValueSet_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_msgs::ValueSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "281ab05df668dd70c6f78f89e9a412a0";
  }

  static const char* value(const ::autoware_msgs::ValueSet_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x281ab05df668dd70ULL;
  static const uint64_t static_value2 = 0xc6f78f89e9a412a0ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_msgs::ValueSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_msgs/ValueSet";
  }

  static const char* value(const ::autoware_msgs::ValueSet_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_msgs::ValueSet_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 center\n"
"int32 range\n"
;
  }

  static const char* value(const ::autoware_msgs::ValueSet_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_msgs::ValueSet_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.center);
      stream.next(m.range);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ValueSet_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_msgs::ValueSet_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_msgs::ValueSet_<ContainerAllocator>& v)
  {
    s << indent << "center: ";
    Printer<int32_t>::stream(s, indent + "  ", v.center);
    s << indent << "range: ";
    Printer<int32_t>::stream(s, indent + "  ", v.range);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MSGS_MESSAGE_VALUESET_H
