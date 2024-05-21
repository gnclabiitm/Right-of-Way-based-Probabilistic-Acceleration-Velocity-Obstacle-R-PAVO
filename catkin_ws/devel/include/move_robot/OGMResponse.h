// Generated by gencpp from file move_robot/OGMResponse.msg
// DO NOT EDIT!


#ifndef MOVE_ROBOT_MESSAGE_OGMRESPONSE_H
#define MOVE_ROBOT_MESSAGE_OGMRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace move_robot
{
template <class ContainerAllocator>
struct OGMResponse_
{
  typedef OGMResponse_<ContainerAllocator> Type;

  OGMResponse_()
    : dummy()  {
    }
  OGMResponse_(const ContainerAllocator& _alloc)
    : dummy(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _dummy_type;
  _dummy_type dummy;





  typedef boost::shared_ptr< ::move_robot::OGMResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::move_robot::OGMResponse_<ContainerAllocator> const> ConstPtr;

}; // struct OGMResponse_

typedef ::move_robot::OGMResponse_<std::allocator<void> > OGMResponse;

typedef boost::shared_ptr< ::move_robot::OGMResponse > OGMResponsePtr;
typedef boost::shared_ptr< ::move_robot::OGMResponse const> OGMResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::move_robot::OGMResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::move_robot::OGMResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::move_robot::OGMResponse_<ContainerAllocator1> & lhs, const ::move_robot::OGMResponse_<ContainerAllocator2> & rhs)
{
  return lhs.dummy == rhs.dummy;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::move_robot::OGMResponse_<ContainerAllocator1> & lhs, const ::move_robot::OGMResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace move_robot

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::move_robot::OGMResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_robot::OGMResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::move_robot::OGMResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::move_robot::OGMResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_robot::OGMResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_robot::OGMResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::move_robot::OGMResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5eab3f8fe195848369929fb790db61c1";
  }

  static const char* value(const ::move_robot::OGMResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5eab3f8fe1958483ULL;
  static const uint64_t static_value2 = 0x69929fb790db61c1ULL;
};

template<class ContainerAllocator>
struct DataType< ::move_robot::OGMResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "move_robot/OGMResponse";
  }

  static const char* value(const ::move_robot::OGMResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::move_robot::OGMResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string dummy\n"
;
  }

  static const char* value(const ::move_robot::OGMResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::move_robot::OGMResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.dummy);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OGMResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::move_robot::OGMResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::move_robot::OGMResponse_<ContainerAllocator>& v)
  {
    s << indent << "dummy: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.dummy);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVE_ROBOT_MESSAGE_OGMRESPONSE_H