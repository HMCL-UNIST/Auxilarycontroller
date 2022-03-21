// Generated by gencpp from file autoware_config_msgs/ConfigCarDPM.msg
// DO NOT EDIT!


#ifndef AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGCARDPM_H
#define AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGCARDPM_H


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
struct ConfigCarDPM_
{
  typedef ConfigCarDPM_<ContainerAllocator> Type;

  ConfigCarDPM_()
    : header()
    , score_threshold(0.0)
    , group_threshold(0.0)
    , Lambda(0)
    , num_cells(0)
    , num_bins(0)  {
    }
  ConfigCarDPM_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , score_threshold(0.0)
    , group_threshold(0.0)
    , Lambda(0)
    , num_cells(0)
    , num_bins(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _score_threshold_type;
  _score_threshold_type score_threshold;

   typedef float _group_threshold_type;
  _group_threshold_type group_threshold;

   typedef int32_t _Lambda_type;
  _Lambda_type Lambda;

   typedef int32_t _num_cells_type;
  _num_cells_type num_cells;

   typedef int32_t _num_bins_type;
  _num_bins_type num_bins;





  typedef boost::shared_ptr< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigCarDPM_

typedef ::autoware_config_msgs::ConfigCarDPM_<std::allocator<void> > ConfigCarDPM;

typedef boost::shared_ptr< ::autoware_config_msgs::ConfigCarDPM > ConfigCarDPMPtr;
typedef boost::shared_ptr< ::autoware_config_msgs::ConfigCarDPM const> ConfigCarDPMConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator1> & lhs, const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.score_threshold == rhs.score_threshold &&
    lhs.group_threshold == rhs.group_threshold &&
    lhs.Lambda == rhs.Lambda &&
    lhs.num_cells == rhs.num_cells &&
    lhs.num_bins == rhs.num_bins;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator1> & lhs, const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_config_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4aad5f13bbefe1a8707af2b040e45167";
  }

  static const char* value(const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4aad5f13bbefe1a8ULL;
  static const uint64_t static_value2 = 0x707af2b040e45167ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_config_msgs/ConfigCarDPM";
  }

  static const char* value(const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"float32 score_threshold\n"
"float32 group_threshold\n"
"int32 Lambda\n"
"int32 num_cells\n"
"int32 num_bins\n"
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

  static const char* value(const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.score_threshold);
      stream.next(m.group_threshold);
      stream.next(m.Lambda);
      stream.next(m.num_cells);
      stream.next(m.num_bins);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigCarDPM_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_config_msgs::ConfigCarDPM_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "score_threshold: ";
    Printer<float>::stream(s, indent + "  ", v.score_threshold);
    s << indent << "group_threshold: ";
    Printer<float>::stream(s, indent + "  ", v.group_threshold);
    s << indent << "Lambda: ";
    Printer<int32_t>::stream(s, indent + "  ", v.Lambda);
    s << indent << "num_cells: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_cells);
    s << indent << "num_bins: ";
    Printer<int32_t>::stream(s, indent + "  ", v.num_bins);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGCARDPM_H
