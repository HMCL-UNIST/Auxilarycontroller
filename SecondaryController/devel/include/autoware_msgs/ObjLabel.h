// Generated by gencpp from file autoware_msgs/ObjLabel.msg
// DO NOT EDIT!


#ifndef AUTOWARE_MSGS_MESSAGE_OBJLABEL_H
#define AUTOWARE_MSGS_MESSAGE_OBJLABEL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>

namespace autoware_msgs
{
template <class ContainerAllocator>
struct ObjLabel_
{
  typedef ObjLabel_<ContainerAllocator> Type;

  ObjLabel_()
    : header()
    , type()
    , obj_id()
    , reprojected_pos()  {
    }
  ObjLabel_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , type(_alloc)
    , obj_id(_alloc)
    , reprojected_pos(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  _type_type type;

   typedef std::vector<int32_t, typename ContainerAllocator::template rebind<int32_t>::other >  _obj_id_type;
  _obj_id_type obj_id;

   typedef std::vector< ::geometry_msgs::Point_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::Point_<ContainerAllocator> >::other >  _reprojected_pos_type;
  _reprojected_pos_type reprojected_pos;





  typedef boost::shared_ptr< ::autoware_msgs::ObjLabel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_msgs::ObjLabel_<ContainerAllocator> const> ConstPtr;

}; // struct ObjLabel_

typedef ::autoware_msgs::ObjLabel_<std::allocator<void> > ObjLabel;

typedef boost::shared_ptr< ::autoware_msgs::ObjLabel > ObjLabelPtr;
typedef boost::shared_ptr< ::autoware_msgs::ObjLabel const> ObjLabelConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_msgs::ObjLabel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_msgs::ObjLabel_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_msgs::ObjLabel_<ContainerAllocator1> & lhs, const ::autoware_msgs::ObjLabel_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.type == rhs.type &&
    lhs.obj_id == rhs.obj_id &&
    lhs.reprojected_pos == rhs.reprojected_pos;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_msgs::ObjLabel_<ContainerAllocator1> & lhs, const ::autoware_msgs::ObjLabel_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::ObjLabel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_msgs::ObjLabel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::ObjLabel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_msgs::ObjLabel_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::ObjLabel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_msgs::ObjLabel_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_msgs::ObjLabel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1908c8a3e1598adc90838e6b6436a771";
  }

  static const char* value(const ::autoware_msgs::ObjLabel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1908c8a3e1598adcULL;
  static const uint64_t static_value2 = 0x90838e6b6436a771ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_msgs::ObjLabel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_msgs/ObjLabel";
  }

  static const char* value(const ::autoware_msgs::ObjLabel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_msgs::ObjLabel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"string type\n"
"int32[] obj_id\n"
"geometry_msgs/Point[] reprojected_pos\n"
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
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::autoware_msgs::ObjLabel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_msgs::ObjLabel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.type);
      stream.next(m.obj_id);
      stream.next(m.reprojected_pos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ObjLabel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_msgs::ObjLabel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_msgs::ObjLabel_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "obj_id[]" << std::endl;
    for (size_t i = 0; i < v.obj_id.size(); ++i)
    {
      s << indent << "  obj_id[" << i << "]: ";
      Printer<int32_t>::stream(s, indent + "  ", v.obj_id[i]);
    }
    s << indent << "reprojected_pos[]" << std::endl;
    for (size_t i = 0; i < v.reprojected_pos.size(); ++i)
    {
      s << indent << "  reprojected_pos[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "    ", v.reprojected_pos[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_MSGS_MESSAGE_OBJLABEL_H
