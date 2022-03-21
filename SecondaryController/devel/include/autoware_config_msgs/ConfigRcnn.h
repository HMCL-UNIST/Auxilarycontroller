// Generated by gencpp from file autoware_config_msgs/ConfigRcnn.msg
// DO NOT EDIT!


#ifndef AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGRCNN_H
#define AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGRCNN_H


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
struct ConfigRcnn_
{
  typedef ConfigRcnn_<ContainerAllocator> Type;

  ConfigRcnn_()
    : header()
    , score_threshold(0.0)
    , group_threshold(0.0)
    , slices_overlap(0.0)
    , b_mean(0.0)
    , g_mean(0.0)
    , r_mean(0.0)
    , image_slices(0)
    , use_gpu(0)
    , gpu_device_id(0)  {
    }
  ConfigRcnn_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , score_threshold(0.0)
    , group_threshold(0.0)
    , slices_overlap(0.0)
    , b_mean(0.0)
    , g_mean(0.0)
    , r_mean(0.0)
    , image_slices(0)
    , use_gpu(0)
    , gpu_device_id(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _score_threshold_type;
  _score_threshold_type score_threshold;

   typedef float _group_threshold_type;
  _group_threshold_type group_threshold;

   typedef float _slices_overlap_type;
  _slices_overlap_type slices_overlap;

   typedef float _b_mean_type;
  _b_mean_type b_mean;

   typedef float _g_mean_type;
  _g_mean_type g_mean;

   typedef float _r_mean_type;
  _r_mean_type r_mean;

   typedef uint8_t _image_slices_type;
  _image_slices_type image_slices;

   typedef uint8_t _use_gpu_type;
  _use_gpu_type use_gpu;

   typedef uint8_t _gpu_device_id_type;
  _gpu_device_id_type gpu_device_id;





  typedef boost::shared_ptr< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigRcnn_

typedef ::autoware_config_msgs::ConfigRcnn_<std::allocator<void> > ConfigRcnn;

typedef boost::shared_ptr< ::autoware_config_msgs::ConfigRcnn > ConfigRcnnPtr;
typedef boost::shared_ptr< ::autoware_config_msgs::ConfigRcnn const> ConfigRcnnConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator1> & lhs, const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.score_threshold == rhs.score_threshold &&
    lhs.group_threshold == rhs.group_threshold &&
    lhs.slices_overlap == rhs.slices_overlap &&
    lhs.b_mean == rhs.b_mean &&
    lhs.g_mean == rhs.g_mean &&
    lhs.r_mean == rhs.r_mean &&
    lhs.image_slices == rhs.image_slices &&
    lhs.use_gpu == rhs.use_gpu &&
    lhs.gpu_device_id == rhs.gpu_device_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator1> & lhs, const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_config_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f51f68bdedfbe5da5d10ace3c7a60ff0";
  }

  static const char* value(const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf51f68bdedfbe5daULL;
  static const uint64_t static_value2 = 0x5d10ace3c7a60ff0ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_config_msgs/ConfigRcnn";
  }

  static const char* value(const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header  header\n"
"float32 score_threshold #minimum score required to keep the detection [0.0, 1.0] (default 0.6)\n"
"float32 group_threshold #minimum overlap percentage area required to supress detections(NMS threshold) [0.0, 1.0] (default 0.3)\n"
"float32 slices_overlap  #overlap percentage between image slices [0.0, 1.0] (default 0.7)\n"
"float32 b_mean\n"
"float32 g_mean\n"
"float32 r_mean\n"
"uint8   image_slices    #number of times to slice the image and search (1, 100], larger value might improve detection but reduce performance (default 16)\n"
"uint8   use_gpu\n"
"uint8   gpu_device_id\n"
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

  static const char* value(const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.score_threshold);
      stream.next(m.group_threshold);
      stream.next(m.slices_overlap);
      stream.next(m.b_mean);
      stream.next(m.g_mean);
      stream.next(m.r_mean);
      stream.next(m.image_slices);
      stream.next(m.use_gpu);
      stream.next(m.gpu_device_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigRcnn_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_config_msgs::ConfigRcnn_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "score_threshold: ";
    Printer<float>::stream(s, indent + "  ", v.score_threshold);
    s << indent << "group_threshold: ";
    Printer<float>::stream(s, indent + "  ", v.group_threshold);
    s << indent << "slices_overlap: ";
    Printer<float>::stream(s, indent + "  ", v.slices_overlap);
    s << indent << "b_mean: ";
    Printer<float>::stream(s, indent + "  ", v.b_mean);
    s << indent << "g_mean: ";
    Printer<float>::stream(s, indent + "  ", v.g_mean);
    s << indent << "r_mean: ";
    Printer<float>::stream(s, indent + "  ", v.r_mean);
    s << indent << "image_slices: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.image_slices);
    s << indent << "use_gpu: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_gpu);
    s << indent << "gpu_device_id: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gpu_device_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGRCNN_H
