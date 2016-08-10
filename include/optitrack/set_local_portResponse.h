/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/gmilliez/robotpkg/localization/optitrack-genom3/work.elea/templates/ros/client/ros/optitrack/srv/set_local_port.srv
 *
 */


#ifndef OPTITRACK_MESSAGE_SET_LOCAL_PORTRESPONSE_H
#define OPTITRACK_MESSAGE_SET_LOCAL_PORTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace optitrack
{
template <class ContainerAllocator>
struct set_local_portResponse_
{
  typedef set_local_portResponse_<ContainerAllocator> Type;

  set_local_portResponse_()
    : genom_success(false)
    , genom_exdetail()  {
    }
  set_local_portResponse_(const ContainerAllocator& _alloc)
    : genom_success(false)
    , genom_exdetail(_alloc)  {
    }



   typedef uint8_t _genom_success_type;
  _genom_success_type genom_success;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _genom_exdetail_type;
  _genom_exdetail_type genom_exdetail;




  typedef boost::shared_ptr< ::optitrack::set_local_portResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::optitrack::set_local_portResponse_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct set_local_portResponse_

typedef ::optitrack::set_local_portResponse_<std::allocator<void> > set_local_portResponse;

typedef boost::shared_ptr< ::optitrack::set_local_portResponse > set_local_portResponsePtr;
typedef boost::shared_ptr< ::optitrack::set_local_portResponse const> set_local_portResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::optitrack::set_local_portResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::optitrack::set_local_portResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace optitrack

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'optitrack': ['optitrack/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::set_local_portResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::optitrack::set_local_portResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::set_local_portResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::optitrack::set_local_portResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::set_local_portResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::optitrack::set_local_portResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::optitrack::set_local_portResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0d79450287345aef3f3e331856b25242";
  }

  static const char* value(const ::optitrack::set_local_portResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0d79450287345aefULL;
  static const uint64_t static_value2 = 0x3f3e331856b25242ULL;
};

template<class ContainerAllocator>
struct DataType< ::optitrack::set_local_portResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "optitrack/set_local_portResponse";
  }

  static const char* value(const ::optitrack::set_local_portResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::optitrack::set_local_portResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool genom_success\n\
string genom_exdetail\n\
\n\
";
  }

  static const char* value(const ::optitrack::set_local_portResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::optitrack::set_local_portResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.genom_success);
      stream.next(m.genom_exdetail);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct set_local_portResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::optitrack::set_local_portResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::optitrack::set_local_portResponse_<ContainerAllocator>& v)
  {
    s << indent << "genom_success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.genom_success);
    s << indent << "genom_exdetail: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.genom_exdetail);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPTITRACK_MESSAGE_SET_LOCAL_PORTRESPONSE_H
