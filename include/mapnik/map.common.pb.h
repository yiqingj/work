// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: map.common.proto

#ifndef PROTOBUF_map_2ecommon_2eproto__INCLUDED
#define PROTOBUF_map_2ecommon_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2005000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/generated_enum_reflection.h>
// @@protoc_insertion_point(includes)

namespace com {
namespace telenav {
namespace proto {
namespace map {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_map_2ecommon_2eproto();
void protobuf_AssignDesc_map_2ecommon_2eproto();
void protobuf_ShutdownFile_map_2ecommon_2eproto();


enum RoadType {
  RT_UNKNOWN = 0,
  RT_HIGHWAY = 1,
  RT_ARTERIAL = 2,
  RT_LOCAL = 3,
  RT_TERMINAL = 4,
  RT_FREE_RAMP = 5,
  RT_ARTERIAL_RAMP = 6,
  RT_FERRY = 7,
  RT_LOCAL_RAMP = 8,
  RT_SMALL_RAMP = 9,
  RT_PRIVATE = 10,
  RT_NON_NAVIGABLE = 11,
  RT_PEDESTRIAN = 12,
  RT_CYCLEWAY = 13,
  RT_LAYOUT = 14,
  RT_SERVICE = 15
};
bool RoadType_IsValid(int value);
const RoadType RoadType_MIN = RT_UNKNOWN;
const RoadType RoadType_MAX = RT_SERVICE;
const int RoadType_ARRAYSIZE = RoadType_MAX + 1;

const ::google::protobuf::EnumDescriptor* RoadType_descriptor();
inline const ::std::string& RoadType_Name(RoadType value) {
  return ::google::protobuf::internal::NameOfEnum(
    RoadType_descriptor(), value);
}
inline bool RoadType_Parse(
    const ::std::string& name, RoadType* value) {
  return ::google::protobuf::internal::ParseNamedEnum<RoadType>(
    RoadType_descriptor(), name, value);
}
enum MapServiceStatus {
  MAP_OK = 11200,
  MAP_NO_CONTENT = 11204,
  MAP_BAD_REQUEST = 11400,
  MAP_SERVICE_NOT_FOUND = 11404,
  MAP_UNKNOWN_ERROR = 11500,
  MAP_SERVICE_UNAVAILABLE = 11503,
  MAP_SERVICE_TIMEOUT = 11504,
  MAP_DB_ERROR = 11510,
  MAP_DB_CONN_ERROR = 11511,
  MAP_DB_DATA_NOT_FOUND = 11512,
  MAP_DB_DUPLICATE_ENTRY = 11513,
  ROUTE_NOT_FOUND = 11601,
  ROUTE_OD_TOO_CLOSE = 11602,
  ORIGIN_NOT_COVER = 11603,
  DESTINATION_NOT_COVER = 11604,
  ORIGIN_NOT_FOUND = 11605,
  DESTINATION_NOT_FOUND = 11606,
  ORIGIN_DESTINATION_ACROSS_REGION = 11607,
  PEDESTRIAN_ROUTE_TOO_LONG = 11611,
  PEDESTRIAN_ON_HIGHWAY = 11612
};
bool MapServiceStatus_IsValid(int value);
const MapServiceStatus MapServiceStatus_MIN = MAP_OK;
const MapServiceStatus MapServiceStatus_MAX = PEDESTRIAN_ON_HIGHWAY;
const int MapServiceStatus_ARRAYSIZE = MapServiceStatus_MAX + 1;

const ::google::protobuf::EnumDescriptor* MapServiceStatus_descriptor();
inline const ::std::string& MapServiceStatus_Name(MapServiceStatus value) {
  return ::google::protobuf::internal::NameOfEnum(
    MapServiceStatus_descriptor(), value);
}
inline bool MapServiceStatus_Parse(
    const ::std::string& name, MapServiceStatus* value) {
  return ::google::protobuf::internal::ParseNamedEnum<MapServiceStatus>(
    MapServiceStatus_descriptor(), name, value);
}
// ===================================================================


// ===================================================================


// ===================================================================


// @@protoc_insertion_point(namespace_scope)

}  // namespace map
}  // namespace proto
}  // namespace telenav
}  // namespace com

#ifndef SWIG
namespace google {
namespace protobuf {

template <>
inline const EnumDescriptor* GetEnumDescriptor< ::com::telenav::proto::map::RoadType>() {
  return ::com::telenav::proto::map::RoadType_descriptor();
}
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::com::telenav::proto::map::MapServiceStatus>() {
  return ::com::telenav::proto::map::MapServiceStatus_descriptor();
}

}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_map_2ecommon_2eproto__INCLUDED
