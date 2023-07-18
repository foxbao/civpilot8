# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/map/proto/map_junction.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from message.map.proto import map_id_pb2 as message_dot_map_dot_proto_dot_map__id__pb2
from message.map.proto import map_geometry_pb2 as message_dot_map_dot_proto_dot_map__geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/map/proto/map_junction.proto',
  package='civ.hdmap',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n$message/map/proto/map_junction.proto\x12\tciv.hdmap\x1a\x1emessage/map/proto/map_id.proto\x1a$message/map/proto/map_geometry.proto\"\xf3\x01\n\x08Junction\x12\x19\n\x02id\x18\x01 \x01(\x0b\x32\r.civ.hdmap.Id\x12#\n\x07polygon\x18\x02 \x01(\x0b\x32\x12.civ.hdmap.Polygon\x12!\n\noverlap_id\x18\x03 \x03(\x0b\x32\r.civ.hdmap.Id\x12&\n\x04type\x18\x04 \x01(\x0e\x32\x18.civ.hdmap.Junction.Type\"\\\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x0b\n\x07IN_ROAD\x10\x01\x12\x0e\n\nCROSS_ROAD\x10\x02\x12\r\n\tFORK_ROAD\x10\x03\x12\r\n\tMAIN_SIDE\x10\x04\x12\x0c\n\x08\x44\x45\x41\x44_END\x10\x05'
  ,
  dependencies=[message_dot_map_dot_proto_dot_map__id__pb2.DESCRIPTOR,message_dot_map_dot_proto_dot_map__geometry__pb2.DESCRIPTOR,])



_JUNCTION_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='civ.hdmap.Junction.Type',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='IN_ROAD', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CROSS_ROAD', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='FORK_ROAD', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='MAIN_SIDE', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DEAD_END', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=273,
  serialized_end=365,
)
_sym_db.RegisterEnumDescriptor(_JUNCTION_TYPE)


_JUNCTION = _descriptor.Descriptor(
  name='Junction',
  full_name='civ.hdmap.Junction',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='civ.hdmap.Junction.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='civ.hdmap.Junction.polygon', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='overlap_id', full_name='civ.hdmap.Junction.overlap_id', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='type', full_name='civ.hdmap.Junction.type', index=3,
      number=4, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _JUNCTION_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=122,
  serialized_end=365,
)

_JUNCTION.fields_by_name['id'].message_type = message_dot_map_dot_proto_dot_map__id__pb2._ID
_JUNCTION.fields_by_name['polygon'].message_type = message_dot_map_dot_proto_dot_map__geometry__pb2._POLYGON
_JUNCTION.fields_by_name['overlap_id'].message_type = message_dot_map_dot_proto_dot_map__id__pb2._ID
_JUNCTION.fields_by_name['type'].enum_type = _JUNCTION_TYPE
_JUNCTION_TYPE.containing_type = _JUNCTION
DESCRIPTOR.message_types_by_name['Junction'] = _JUNCTION
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Junction = _reflection.GeneratedProtocolMessageType('Junction', (_message.Message,), {
  'DESCRIPTOR' : _JUNCTION,
  '__module__' : 'message.map.proto.map_junction_pb2'
  # @@protoc_insertion_point(class_scope:civ.hdmap.Junction)
  })
_sym_db.RegisterMessage(Junction)


# @@protoc_insertion_point(module_scope)
