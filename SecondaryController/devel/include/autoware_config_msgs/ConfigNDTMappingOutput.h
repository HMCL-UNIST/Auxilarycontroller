// Generated by gencpp from file autoware_config_msgs/ConfigNDTMappingOutput.msg
// DO NOT EDIT!


#ifndef AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGNDTMAPPINGOUTPUT_H
#define AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGNDTMAPPINGOUTPUT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace autoware_config_msgs
{
template <class ContainerAllocator>
struct ConfigNDTMappingOutput_
{
  typedef ConfigNDTMappingOutput_<ContainerAllocator> Type;

  ConfigNDTMappingOutput_()
    : header()
    , filename()
    , filter_res(0.0)  {
    }
  ConfigNDTMappingOutput_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , filename(_alloc)
    , filter_res(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _filename_type;
  _filename_type filename;

   typedef float _filter_res_type;
  _filter_res_type filter_res;





  typedef boost::shared_ptr< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigNDTMappingOutput_

typedef ::autoware_config_msgs::ConfigNDTMappingOutput_<std::allocator<void> > ConfigNDTMappingOutput;

typedef boost::shared_ptr< ::autoware_config_msgs::ConfigNDTMappingOutput > ConfigNDTMappingOutputPtr;
typedef boost::shared_ptr< ::autoware_config_msgs::ConfigNDTMappingOutput const> ConfigNDTMappingOutputConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator1> & lhs, const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.filename == rhs.filename &&
    lhs.filter_res == rhs.filter_res;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator1> & lhs, const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_config_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ac31ee963c2f2d01d1d409a7749c20f6";
  }

  static const char* value(const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xac31ee963c2f2d01ULL;
  static const uint64_t static_value2 = 0xd1d409a7749c20f6ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_config_msgs/ConfigNDTMappingOutput";
  }

  static const char* value(const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"string filename\n"
"float32 filter_res\n"
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
;
  }

  static const char* value(const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.filename);
      stream.next(m.filter_res);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigNDTMappingOutput_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_config_msgs::ConfigNDTMappingOutput_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "filename: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.filename);
    s << indent << "filter_res: ";
    Printer<float>::stream(s, indent + "  ", v.filter_res);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGNDTMAPPINGOUTPUT_H
