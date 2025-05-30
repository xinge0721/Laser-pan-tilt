// Generated by gencpp from file Serial/dateRequest.msg
// DO NOT EDIT!


#ifndef SERIAL_MESSAGE_DATEREQUEST_H
#define SERIAL_MESSAGE_DATEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace Serial
{
template <class ContainerAllocator>
struct dateRequest_
{
  typedef dateRequest_<ContainerAllocator> Type;

  dateRequest_()
    : mood(0)  {
    }
  dateRequest_(const ContainerAllocator& _alloc)
    : mood(0)  {
  (void)_alloc;
    }



   typedef int32_t _mood_type;
  _mood_type mood;





  typedef boost::shared_ptr< ::Serial::dateRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::Serial::dateRequest_<ContainerAllocator> const> ConstPtr;

}; // struct dateRequest_

typedef ::Serial::dateRequest_<std::allocator<void> > dateRequest;

typedef boost::shared_ptr< ::Serial::dateRequest > dateRequestPtr;
typedef boost::shared_ptr< ::Serial::dateRequest const> dateRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::Serial::dateRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::Serial::dateRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::Serial::dateRequest_<ContainerAllocator1> & lhs, const ::Serial::dateRequest_<ContainerAllocator2> & rhs)
{
  return lhs.mood == rhs.mood;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::Serial::dateRequest_<ContainerAllocator1> & lhs, const ::Serial::dateRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace Serial

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::Serial::dateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::Serial::dateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::Serial::dateRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::Serial::dateRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::Serial::dateRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::Serial::dateRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::Serial::dateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1f282c3faf48956454b4ac1f1ae8240a";
  }

  static const char* value(const ::Serial::dateRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1f282c3faf489564ULL;
  static const uint64_t static_value2 = 0x54b4ac1f1ae8240aULL;
};

template<class ContainerAllocator>
struct DataType< ::Serial::dateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Serial/dateRequest";
  }

  static const char* value(const ::Serial::dateRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::Serial::dateRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 mood\n"
;
  }

  static const char* value(const ::Serial::dateRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::Serial::dateRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mood);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct dateRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::Serial::dateRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::Serial::dateRequest_<ContainerAllocator>& v)
  {
    if (false || !indent.empty())
      s << std::endl;
    s << indent << "mood: ";
    Printer<int32_t>::stream(s, indent + "  ", v.mood);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SERIAL_MESSAGE_DATEREQUEST_H
