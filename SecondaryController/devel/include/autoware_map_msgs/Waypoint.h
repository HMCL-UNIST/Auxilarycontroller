// Generated by gencpp from file autoware_map_msgs/Waypoint.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MAP_MSGS_MESSAGE_WAYPOINT_H
#define AUTOWARE_MAP_MSGS_MESSAGE_WAYPOINT_H


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
struct Waypoint_
{
  typedef Waypoint_<ContainerAllocator> Type;

  Waypoint_()
    : waypoint_id(0)
    , point_id(0)
    , velocity(0.0)
    , stop_line(0)
    , left_width(0.0)
    , right_width(0.0)
    , height(0.0)  {
    }
  Waypoint_(const ContainerAllocator& _alloc)
    : waypoint_id(0)
    , point_id(0)
    , velocity(0.0)
    , stop_line(0)
    , left_width(0.0)
    , right_width(0.0)
    , height(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _waypoint_id_type;
  _waypoint_id_type waypoint_id;

   typedef int32_t _point_id_type;
  _point_id_type point_id;

   typedef double _velocity_type;
  _velocity_type velocity;

   typedef int32_t _stop_line_type;
  _stop_line_type stop_line;

   typedef double _left_width_type;
  _left_width_type left_width;

   typedef double _right_width_type;
  _right_width_type right_width;

   typedef double _height_type;
  _height_type height;





  typedef boost::shared_ptr< ::autoware_map_msgs::Waypoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_map_msgs::Waypoint_<ContainerAllocator> const> ConstPtr;

}; // struct Waypoint_

typedef ::autoware_map_msgs::Waypoint_<std::allocator<void> > Waypoint;

typedef boost::shared_ptr< ::autoware_map_msgs::Waypoint > WaypointPtr;
typedef boost::shared_ptr< ::autoware_map_msgs::Waypoint const> WaypointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_map_msgs::Waypoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_map_msgs::Waypoint_<ContainerAllocator1> & lhs, const ::autoware_map_msgs::Waypoint_<ContainerAllocator2> & rhs)
{
  return lhs.waypoint_id == rhs.waypoint_id &&
    lhs.point_id == rhs.point_id &&
    lhs.velocity == rhs.velocity &&
    lhs.stop_line == rhs.stop_line &&
    lhs.left_width == rhs.left_width &&
    lhs.right_width == rhs.right_width &&
    lhs.height == rhs.height;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_map_msgs::Waypoint_<ContainerAllocator1> & lhs, const ::autoware_map_msgs::Waypoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_map_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_map_msgs::Waypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_map_msgs::Waypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_map_msgs::Waypoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6a92fbf1608fe14a2a517fbd332e0c6a";
  }

  static const char* value(const ::autoware_map_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6a92fbf1608fe14aULL;
  static const uint64_t static_value2 = 0x2a517fbd332e0c6aULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_map_msgs/Waypoint";
  }

  static const char* value(const ::autoware_map_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This represents a waypoint in a map. \n"
"\n"
"# Id of this Waypoint object\n"
"int32 waypoint_id\n"
"\n"
"# Id of Point that represents the position of this waypoint\n"
"int32 point_id\n"
"\n"
"# reference velocity of this waypoint. [km/h]\n"
"float64 velocity\n"
"\n"
"# describes whether vehicle must stop at this waypoint\n"
"# no_stop = 0, stop = 1\n"
"int32 stop_line\n"
"\n"
"# distance to left border of the belonging lane in [m]\n"
"float64 left_width\n"
"\n"
"# distance to right border of the belonging lane in [m]\n"
"float64 right_width\n"
"\n"
"# height limit for the vehicle to drive this waypoint [m]\n"
"float64 height\n"
;
  }

  static const char* value(const ::autoware_map_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.waypoint_id);
      stream.next(m.point_id);
      stream.next(m.velocity);
      stream.next(m.stop_line);
      stream.next(m.left_width);
      stream.next(m.right_width);
      stream.next(m.height);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Waypoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_map_msgs::Waypoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_map_msgs::Waypoint_<ContainerAllocator>& v)
  {
    s << indent << "waypoint_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.waypoint_id);
    s << indent << "point_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.point_id);
    s << indent << "velocity: ";
    Printer<double>::stream(s, indent + "  ", v.velocity);
    s << indent << "stop_line: ";
    Printer<int32_t>::stream(s, indent + "  ", v.stop_line);
    s << indent << "left_width: ";
    Printer<double>::stream(s, indent + "  ", v.left_width);
    s << indent << "right_width: ";
    Printer<double>::stream(s, indent + "  ", v.right_width);
    s << indent << "height: ";
    Printer<double>::stream(s, indent + "  ", v.height);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MAP_MSGS_MESSAGE_WAYPOINT_H
