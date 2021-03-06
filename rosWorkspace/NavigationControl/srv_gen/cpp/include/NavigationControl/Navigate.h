/* Auto-generated by genmsg_cpp for file /opt/robosub/rosWorkspace/NavigationControl/srv/Navigate.srv */
#ifndef NAVIGATIONCONTROL_SERVICE_NAVIGATE_H
#define NAVIGATIONCONTROL_SERVICE_NAVIGATE_H
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"

#include "ros/service_traits.h"




namespace NavigationControl
{
template <class ContainerAllocator>
struct NavigateRequest_ : public ros::Message
{
  typedef NavigateRequest_<ContainerAllocator> Type;

  NavigateRequest_()
  : camera_direction(0)
  , desired_x(0)
  , desired_y(0)
  , desired_rotation(0.0)
  {
  }

  NavigateRequest_(const ContainerAllocator& _alloc)
  : camera_direction(0)
  , desired_x(0)
  , desired_y(0)
  , desired_rotation(0.0)
  {
  }

  typedef int8_t _camera_direction_type;
  int8_t camera_direction;

  typedef int16_t _desired_x_type;
  int16_t desired_x;

  typedef int16_t _desired_y_type;
  int16_t desired_y;

  typedef float _desired_rotation_type;
  float desired_rotation;


private:
  static const char* __s_getDataType_() { return "NavigationControl/NavigateRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "e21d3132f2554653d6ebf201738a4ca5"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "256da788932aac90025db21f7d359b9e"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 camera_direction\n\
int16 desired_x\n\
int16 desired_y\n\
float32 desired_rotation\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, camera_direction);
    ros::serialization::serialize(stream, desired_x);
    ros::serialization::serialize(stream, desired_y);
    ros::serialization::serialize(stream, desired_rotation);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, camera_direction);
    ros::serialization::deserialize(stream, desired_x);
    ros::serialization::deserialize(stream, desired_y);
    ros::serialization::deserialize(stream, desired_rotation);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(camera_direction);
    size += ros::serialization::serializationLength(desired_x);
    size += ros::serialization::serializationLength(desired_y);
    size += ros::serialization::serializationLength(desired_rotation);
    return size;
  }

  typedef boost::shared_ptr< ::NavigationControl::NavigateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::NavigationControl::NavigateRequest_<ContainerAllocator>  const> ConstPtr;
}; // struct NavigateRequest
typedef  ::NavigationControl::NavigateRequest_<std::allocator<void> > NavigateRequest;

typedef boost::shared_ptr< ::NavigationControl::NavigateRequest> NavigateRequestPtr;
typedef boost::shared_ptr< ::NavigationControl::NavigateRequest const> NavigateRequestConstPtr;


template <class ContainerAllocator>
struct NavigateResponse_ : public ros::Message
{
  typedef NavigateResponse_<ContainerAllocator> Type;

  NavigateResponse_()
  : result(0)
  {
  }

  NavigateResponse_(const ContainerAllocator& _alloc)
  : result(0)
  {
  }

  typedef int8_t _result_type;
  int8_t result;


private:
  static const char* __s_getDataType_() { return "NavigationControl/NavigateResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "4414c67819626a1b8e0f043a9a0d6c9a"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "256da788932aac90025db21f7d359b9e"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int8 result\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, result);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, result);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(result);
    return size;
  }

  typedef boost::shared_ptr< ::NavigationControl::NavigateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::NavigationControl::NavigateResponse_<ContainerAllocator>  const> ConstPtr;
}; // struct NavigateResponse
typedef  ::NavigationControl::NavigateResponse_<std::allocator<void> > NavigateResponse;

typedef boost::shared_ptr< ::NavigationControl::NavigateResponse> NavigateResponsePtr;
typedef boost::shared_ptr< ::NavigationControl::NavigateResponse const> NavigateResponseConstPtr;

struct Navigate
{

typedef NavigateRequest Request;
typedef NavigateResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct Navigate
} // namespace NavigationControl

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::NavigationControl::NavigateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e21d3132f2554653d6ebf201738a4ca5";
  }

  static const char* value(const  ::NavigationControl::NavigateRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe21d3132f2554653ULL;
  static const uint64_t static_value2 = 0xd6ebf201738a4ca5ULL;
};

template<class ContainerAllocator>
struct DataType< ::NavigationControl::NavigateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "NavigationControl/NavigateRequest";
  }

  static const char* value(const  ::NavigationControl::NavigateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::NavigationControl::NavigateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 camera_direction\n\
int16 desired_x\n\
int16 desired_y\n\
float32 desired_rotation\n\
\n\
";
  }

  static const char* value(const  ::NavigationControl::NavigateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::NavigationControl::NavigateRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator>
struct MD5Sum< ::NavigationControl::NavigateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4414c67819626a1b8e0f043a9a0d6c9a";
  }

  static const char* value(const  ::NavigationControl::NavigateResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4414c67819626a1bULL;
  static const uint64_t static_value2 = 0x8e0f043a9a0d6c9aULL;
};

template<class ContainerAllocator>
struct DataType< ::NavigationControl::NavigateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "NavigationControl/NavigateResponse";
  }

  static const char* value(const  ::NavigationControl::NavigateResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::NavigationControl::NavigateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 result\n\
\n\
\n\
";
  }

  static const char* value(const  ::NavigationControl::NavigateResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::NavigationControl::NavigateResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::NavigationControl::NavigateRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.camera_direction);
    stream.next(m.desired_x);
    stream.next(m.desired_y);
    stream.next(m.desired_rotation);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct NavigateRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::NavigationControl::NavigateResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct NavigateResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<NavigationControl::Navigate> {
  static const char* value() 
  {
    return "256da788932aac90025db21f7d359b9e";
  }

  static const char* value(const NavigationControl::Navigate&) { return value(); } 
};

template<>
struct DataType<NavigationControl::Navigate> {
  static const char* value() 
  {
    return "NavigationControl/Navigate";
  }

  static const char* value(const NavigationControl::Navigate&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<NavigationControl::NavigateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "256da788932aac90025db21f7d359b9e";
  }

  static const char* value(const NavigationControl::NavigateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<NavigationControl::NavigateRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "NavigationControl/Navigate";
  }

  static const char* value(const NavigationControl::NavigateRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<NavigationControl::NavigateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "256da788932aac90025db21f7d359b9e";
  }

  static const char* value(const NavigationControl::NavigateResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<NavigationControl::NavigateResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "NavigationControl/Navigate";
  }

  static const char* value(const NavigationControl::NavigateResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // NAVIGATIONCONTROL_SERVICE_NAVIGATE_H

