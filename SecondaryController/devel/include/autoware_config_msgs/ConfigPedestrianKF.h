// Generated by gencpp from file autoware_config_msgs/ConfigPedestrianKF.msg
// DO NOT EDIT!


#ifndef AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGPEDESTRIANKF_H
#define AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGPEDESTRIANKF_H


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
struct ConfigPedestrianKF_
{
  typedef ConfigPedestrianKF_<ContainerAllocator> Type;

  ConfigPedestrianKF_()
    : header()
    , initial_lifespan(0)
    , default_lifespan(0)
    , noise_covariance(0.0)
    , measurement_noise_covariance(0.0)
    , error_estimate_covariance(0.0)
    , percentage_of_overlapping(0.0)
    , orb_features(0)
    , use_orb(0)  {
    }
  ConfigPedestrianKF_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , initial_lifespan(0)
    , default_lifespan(0)
    , noise_covariance(0.0)
    , measurement_noise_covariance(0.0)
    , error_estimate_covariance(0.0)
    , percentage_of_overlapping(0.0)
    , orb_features(0)
    , use_orb(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _initial_lifespan_type;
  _initial_lifespan_type initial_lifespan;

   typedef int32_t _default_lifespan_type;
  _default_lifespan_type default_lifespan;

   typedef float _noise_covariance_type;
  _noise_covariance_type noise_covariance;

   typedef float _measurement_noise_covariance_type;
  _measurement_noise_covariance_type measurement_noise_covariance;

   typedef float _error_estimate_covariance_type;
  _error_estimate_covariance_type error_estimate_covariance;

   typedef float _percentage_of_overlapping_type;
  _percentage_of_overlapping_type percentage_of_overlapping;

   typedef int32_t _orb_features_type;
  _orb_features_type orb_features;

   typedef int32_t _use_orb_type;
  _use_orb_type use_orb;





  typedef boost::shared_ptr< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> const> ConstPtr;

}; // struct ConfigPedestrianKF_

typedef ::autoware_config_msgs::ConfigPedestrianKF_<std::allocator<void> > ConfigPedestrianKF;

typedef boost::shared_ptr< ::autoware_config_msgs::ConfigPedestrianKF > ConfigPedestrianKFPtr;
typedef boost::shared_ptr< ::autoware_config_msgs::ConfigPedestrianKF const> ConfigPedestrianKFConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator1> & lhs, const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.initial_lifespan == rhs.initial_lifespan &&
    lhs.default_lifespan == rhs.default_lifespan &&
    lhs.noise_covariance == rhs.noise_covariance &&
    lhs.measurement_noise_covariance == rhs.measurement_noise_covariance &&
    lhs.error_estimate_covariance == rhs.error_estimate_covariance &&
    lhs.percentage_of_overlapping == rhs.percentage_of_overlapping &&
    lhs.orb_features == rhs.orb_features &&
    lhs.use_orb == rhs.use_orb;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator1> & lhs, const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace autoware_config_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "35fb5980cbba7f237457ede929c58aa9";
  }

  static const char* value(const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x35fb5980cbba7f23ULL;
  static const uint64_t static_value2 = 0x7457ede929c58aa9ULL;
};

template<class ContainerAllocator>
struct DataType< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "autoware_config_msgs/ConfigPedestrianKF";
  }

  static const char* value(const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"int32 initial_lifespan\n"
"int32 default_lifespan\n"
"float32 noise_covariance\n"
"float32 measurement_noise_covariance\n"
"float32 error_estimate_covariance\n"
"float32 percentage_of_overlapping\n"
"int32 orb_features\n"
"int32 use_orb\n"
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

  static const char* value(const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.initial_lifespan);
      stream.next(m.default_lifespan);
      stream.next(m.noise_covariance);
      stream.next(m.measurement_noise_covariance);
      stream.next(m.error_estimate_covariance);
      stream.next(m.percentage_of_overlapping);
      stream.next(m.orb_features);
      stream.next(m.use_orb);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ConfigPedestrianKF_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::autoware_config_msgs::ConfigPedestrianKF_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "initial_lifespan: ";
    Printer<int32_t>::stream(s, indent + "  ", v.initial_lifespan);
    s << indent << "default_lifespan: ";
    Printer<int32_t>::stream(s, indent + "  ", v.default_lifespan);
    s << indent << "noise_covariance: ";
    Printer<float>::stream(s, indent + "  ", v.noise_covariance);
    s << indent << "measurement_noise_covariance: ";
    Printer<float>::stream(s, indent + "  ", v.measurement_noise_covariance);
    s << indent << "error_estimate_covariance: ";
    Printer<float>::stream(s, indent + "  ", v.error_estimate_covariance);
    s << indent << "percentage_of_overlapping: ";
    Printer<float>::stream(s, indent + "  ", v.percentage_of_overlapping);
    s << indent << "orb_features: ";
    Printer<int32_t>::stream(s, indent + "  ", v.orb_features);
    s << indent << "use_orb: ";
    Printer<int32_t>::stream(s, indent + "  ", v.use_orb);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUTOWARE_CONFIG_MSGS_MESSAGE_CONFIGPEDESTRIANKF_H
