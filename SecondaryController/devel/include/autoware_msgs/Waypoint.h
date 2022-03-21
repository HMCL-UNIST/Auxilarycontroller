// Generated by gencpp from file autoware_msgs/Waypoint.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MSGS_MESSAGE_WAYPOINT_H
#define AUTOWARE_MSGS_MESSAGE_WAYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/DTLane.h>
#include <autoware_msgs/WaypointState.h>

namespace autoware_msgs
{
template <class ContainerAllocator>
struct Waypoint_
{
  typedef Waypoint_<ContainerAllocator> Type;

  Waypoint_()
    : gid(0)
    , lid(0)
    , pose()
    , twist()
    , dtlane()
    , change_flag(0)
    , wpstate()
    , lane_id(0)
    , left_lane_id(0)
    , right_lane_id(0)
    , stop_line_id(0)
    , cost(0.0)
    , time_cost(0.0)
    , direction(0)  {
    }
  Waypoint_(const ContainerAllocator& _alloc)
    : gid(0)
    , lid(0)
    , pose(_alloc)
    , twist(_alloc)
    , dtlane(_alloc)
    , change_flag(0)
    , wpstate(_alloc)
    , lane_id(0)
    , left_lane_id(0)
    , right_lane_id(0)
    , stop_line_id(0)
    , cost(0.0)
    , time_cost(0.0)
    , direction(0)  {
  (void)_alloc;
    }



   typedef int32_t _gid_type;
  _gid_type gid;

   typedef int32_t _lid_type;
  _lid_type lid;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::TwistStamped_<ContainerAllocator>  _twist_type;
  _twist_type twist;

   typedef  ::autoware_msgs::DTLane_<ContainerAllocator>  _dtlane_type;
  _dtlane_type dtlane;

   typedef int32_t _change_flag_type;
  _change_flag_type change_flag;

   typedef  ::autoware_msgs::WaypointState_<ContainerAllocator>  _wpstate_type;
  _wpstate_type wpstate;

   typedef uint32_t _lane_id_type;
  _lane_id_type lane_id;

   typedef uint32_t _left_lane_id_type;
  _left_lane_id_type left_lane_id;

   typedef uint32_t _right_lane_id_type;
  _right_lane_id_type right_lane_id;

   typedef uint32_t _stop_line_id_type;
  _stop_line_id_type stop_line_id;

   typedef float _cost_type;
  _cost_type cost;

   typedef float _time_cost_type;
  _time_cost_type time_cost;

   typedef uint32_t _direction_type;
  _direction_type direction;





  typedef boost::shared_ptr< ::autoware_msgs::Waypoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_msgs::Waypoint_<ContainerAllocator> const> ConstPtr;

}; // struct Waypoint_

typedef ::autoware_msgs::Waypoint_<std::allocator<void> > Waypoint;

typedef boost::shared_ptr< ::autoware_msgs::Waypoint > WaypointPtr;
typedef boost::shared_ptr< ::autoware_msgs::Waypoint const> WaypointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_msgs::Waypoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_msgs::Waypoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_msgs::Waypoint_<ContainerAllocator1> & lhs, const ::autoware_msgs::Waypoint_<ContainerAllocator2> & rhs)
{
  return lhs.gid == rhs.gid &&
    lhs.lid == rhs.lid &&
    lhs.pose == rhs.pose &&
    lhs.twist == rhs.twist &&
    lhs.dtlane == rhs.dtlane &&
    lhs.change_flag == rhs.change_flag &&
    lhs.wpstate == rhs.wpstate &&
    lhs.lane_id == rhs.lane_id &&
    lhs.left_lane_id == rhs.left_lane_id &&
    lhs.right_lane_id == rhs.right_lane_id &&
    lhs.stop_line_id == rhs.stop_line_id &&
    lhs.cost == rhs.cost &&
    lhs.time_cost == rhs.time_cost &&
    lhs.direction == rhs.direction;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_msgs::Waypoint_<ContainerAllocator1> & lhs, const ::autoware_msgs::Waypoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::Waypoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::Waypoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::Waypoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::Waypoint_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::Waypoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::Waypoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f980f8144ba8190e8db52ab486d506e2";
  }

  static const char* value(const ::autoware_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf980f8144ba8190eULL;
  static const uint64_t static_value2 = 0x8db52ab486d506e2ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_msgs/Waypoint";
  }

  static const char* value(const ::autoware_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_msgs::Waypoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# global id\n"
"int32 gid \n"
"# local id\n"
"int32 lid\n"
"geometry_msgs/PoseStamped pose\n"
"geometry_msgs/TwistStamped twist\n"
"DTLane dtlane\n"
"int32 change_flag\n"
"WaypointState wpstate\n"
"\n"
"uint32 lane_id\n"
"uint32 left_lane_id\n"
"uint32 right_lane_id\n"
"uint32 stop_line_id\n"
"float32 cost\n"
"float32 time_cost\n"
"\n"
"# Lane Direction\n"
"# FORWARD        = 0\n"
"# FORWARD_LEFT       = 1\n"
"# FORWARD_RIGHT      = 2\n"
"# BACKWARD        = 3 \n"
"# BACKWARD_LEFT      = 4\n"
"# BACKWARD_RIGHT    = 5\n"
"# STANDSTILL       = 6\n"
"uint32 direction\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/TwistStamped\n"
"# A twist with reference coordinate frame and timestamp\n"
"Header header\n"
"Twist twist\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Twist\n"
"# This expresses velocity in free space broken into its linear and angular parts.\n"
"Vector3  linear\n"
"Vector3  angular\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"================================================================================\n"
"MSG: autoware_msgs/DTLane\n"
"float64 dist\n"
"float64 dir\n"
"float64 apara\n"
"float64 r\n"
"float64 slope\n"
"float64 cant\n"
"float64 lw\n"
"float64 rw\n"
"\n"
"================================================================================\n"
"MSG: autoware_msgs/WaypointState\n"
"int32 aid\n"
"uint8 NULLSTATE=0\n"
"\n"
"# lanechange\n"
"uint8 lanechange_state\n"
"\n"
"# bilinker\n"
"uint8 steering_state\n"
"uint8 STR_LEFT=1\n"
"uint8 STR_RIGHT=2\n"
"uint8 STR_STRAIGHT=3\n"
"uint8 STR_BACK=4\n"
"\n"
"uint8 accel_state\n"
"\n"
"uint8 stop_state\n"
"# 1 is stopline, 2 is stop which can only be released manually.\n"
"uint8 TYPE_STOPLINE=1\n"
"uint8 TYPE_STOP=2\n"
"\n"
"uint8 event_state\n"
"uint8 TYPE_EVENT_NULL = 0\n"
"uint8 TYPE_EVENT_GOAL = 1\n"
"uint8 TYPE_EVENT_MIDDLE_GOAL = 2\n"
"uint8 TYPE_EVENT_POSITION_STOP = 3\n"
"uint8 TYPE_EVENT_BUS_STOP = 4\n"
"uint8 TYPE_EVENT_PARKING = 5\n"
;
  }

  static const char* value(const ::autoware_msgs::Waypoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_msgs::Waypoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gid);
      stream.next(m.lid);
      stream.next(m.pose);
      stream.next(m.twist);
      stream.next(m.dtlane);
      stream.next(m.change_flag);
      stream.next(m.wpstate);
      stream.next(m.lane_id);
      stream.next(m.left_lane_id);
      stream.next(m.right_lane_id);
      stream.next(m.stop_line_id);
      stream.next(m.cost);
      stream.next(m.time_cost);
      stream.next(m.direction);
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
struct Printer< ::autoware_msgs::Waypoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_msgs::Waypoint_<ContainerAllocator>& v)
  {
    s << indent << "gid: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gid);
    s << indent << "lid: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lid);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "twist: ";
    s << std::endl;
    Printer< ::geometry_msgs::TwistStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.twist);
    s << indent << "dtlane: ";
    s << std::endl;
    Printer< ::autoware_msgs::DTLane_<ContainerAllocator> >::stream(s, indent + "  ", v.dtlane);
    s << indent << "change_flag: ";
    Printer<int32_t>::stream(s, indent + "  ", v.change_flag);
    s << indent << "wpstate: ";
    s << std::endl;
    Printer< ::autoware_msgs::WaypointState_<ContainerAllocator> >::stream(s, indent + "  ", v.wpstate);
    s << indent << "lane_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.lane_id);
    s << indent << "left_lane_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.left_lane_id);
    s << indent << "right_lane_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.right_lane_id);
    s << indent << "stop_line_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.stop_line_id);
    s << indent << "cost: ";
    Printer<float>::stream(s, indent + "  ", v.cost);
    s << indent << "time_cost: ";
    Printer<float>::stream(s, indent + "  ", v.time_cost);
    s << indent << "direction: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.direction);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MSGS_MESSAGE_WAYPOINT_H
