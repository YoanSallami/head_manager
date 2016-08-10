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
 * Auto-generated by gensrv_cpp from file /home/gmilliez/robotpkg/localization/optitrack-genom3/work.elea/templates/ros/client/ros/optitrack/srv/get_export.srv
 *
 */


#ifndef OPTITRACK_MESSAGE_GET_EXPORT_H
#define OPTITRACK_MESSAGE_GET_EXPORT_H

#include <ros/service_traits.h>


#include <optitrack/get_exportRequest.h>
#include <optitrack/get_exportResponse.h>


namespace optitrack
{

struct get_export
{

typedef get_exportRequest Request;
typedef get_exportResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct get_export
} // namespace optitrack


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::optitrack::get_export > {
  static const char* value()
  {
    return "d1a085cb27982cde459f55828257a45e";
  }

  static const char* value(const ::optitrack::get_export&) { return value(); }
};

template<>
struct DataType< ::optitrack::get_export > {
  static const char* value()
  {
    return "optitrack/get_export";
  }

  static const char* value(const ::optitrack::get_export&) { return value(); }
};


// service_traits::MD5Sum< ::optitrack::get_exportRequest> should match 
// service_traits::MD5Sum< ::optitrack::get_export > 
template<>
struct MD5Sum< ::optitrack::get_exportRequest>
{
  static const char* value()
  {
    return MD5Sum< ::optitrack::get_export >::value();
  }
  static const char* value(const ::optitrack::get_exportRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::optitrack::get_exportRequest> should match 
// service_traits::DataType< ::optitrack::get_export > 
template<>
struct DataType< ::optitrack::get_exportRequest>
{
  static const char* value()
  {
    return DataType< ::optitrack::get_export >::value();
  }
  static const char* value(const ::optitrack::get_exportRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::optitrack::get_exportResponse> should match 
// service_traits::MD5Sum< ::optitrack::get_export > 
template<>
struct MD5Sum< ::optitrack::get_exportResponse>
{
  static const char* value()
  {
    return MD5Sum< ::optitrack::get_export >::value();
  }
  static const char* value(const ::optitrack::get_exportResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::optitrack::get_exportResponse> should match 
// service_traits::DataType< ::optitrack::get_export > 
template<>
struct DataType< ::optitrack::get_exportResponse>
{
  static const char* value()
  {
    return DataType< ::optitrack::get_export >::value();
  }
  static const char* value(const ::optitrack::get_exportResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // OPTITRACK_MESSAGE_GET_EXPORT_H
