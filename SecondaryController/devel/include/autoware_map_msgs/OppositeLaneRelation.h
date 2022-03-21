// Generated by gencpp from file autoware_map_msgs/OppositeLaneRelation.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MAP_MSGS_MESSAGE_OPPOSITELANERELATION_H
#define AUTOWARE_MAP_MSGS_MESSAGE_OPPOSITELANERELATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace autoware_map_msgs
{
template <class ContainerAllocator>
struct OppositeLaneRelation_
{
  typedef OppositeLaneRelation_<ContainerAllocator> Type;

  OppositeLaneRelation_()
    : lane_id(0)
    , opposite_lane_id(0)  {
    }
  OppositeLaneRelation_(const ContainerAllocator& _alloc)
    : lane_id(0)
    , opposite_lane_id(0)  {
  (void)_alloc;
    }



   typedef int32_t _lane_id_type;
  _lane_id_type lane_id;

   typedef int32_t _opposite_lane_id_type;
  _opposite_lane_id_type opposite_lane_id;





  typedef boost::shared_ptr< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> const> ConstPtr;

}; // struct OppositeLaneRelation_

typedef ::autoware_map_msgs::OppositeLaneRelation_<std::allocator<void> > OppositeLaneRelation;

typedef boost::shared_ptr< ::autoware_map_msgs::OppositeLaneRelation > OppositeLaneRelationPtr;
typedef boost::shared_ptr< ::autoware_map_msgs::OppositeLaneRelation const> OppositeLaneRelationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator1> & lhs, const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator2> & rhs)
{
  return lhs.lane_id == rhs.lane_id &&
    lhs.opposite_lane_id == rhs.opposite_lane_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator1> & lhs, const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_map_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1cd180a4c94ee476a03a85a837390bf7";
  }

  static const char* value(const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1cd180a4c94ee476ULL;
  static const uint64_t static_value2 = 0xa03a85a837390bf7ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_map_msgs/OppositeLaneRelation";
  }

  static const char* value(const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This represents opposite lanes in a map.\n"
"\n"
"# Id of refering Lane object\n"
"int32 lane_id\n"
"\n"
"# Id of Lane object that is in opposite direction against refering lane. \n"
"int32 opposite_lane_id\n"
;
  }

  static const char* value(const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.lane_id);
      stream.next(m.opposite_lane_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OppositeLaneRelation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_map_msgs::OppositeLaneRelation_<ContainerAllocator>& v)
  {
    s << indent << "lane_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lane_id);
    s << indent << "opposite_lane_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.opposite_lane_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MAP_MSGS_MESSAGE_OPPOSITELANERELATION_H
