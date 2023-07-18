# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/map/proto/map_crosswalk.proto
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
  name='message/map/proto/map_crosswalk.proto',
  package='civ.hdmap',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n%message/map/proto/map_crosswalk.proto\x12\tciv.hdmap\x1a\x1emessage/map/proto/map_id.proto\x1a$message/map/proto/map_geometry.proto\"n\n\tCrosswalk\x12\x19\n\x02id\x18\x01 \x01(\x0b\x32\r.civ.hdmap.Id\x12#\n\x07polygon\x18\x02 \x01(\x0b\x32\x12.civ.hdmap.Polygon\x12!\n\noverlap_id\x18\x03 \x03(\x0b\x32\r.civ.hdmap.Id'
  ,
  dependencies=[message_dot_map_dot_proto_dot_map__id__pb2.DESCRIPTOR,message_dot_map_dot_proto_dot_map__geometry__pb2.DESCRIPTOR,])




_CROSSWALK = _descriptor.Descriptor(
  name='Crosswalk',
  full_name='civ.hdmap.Crosswalk',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='civ.hdmap.Crosswalk.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='polygon', full_name='civ.hdmap.Crosswalk.polygon', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='overlap_id', full_name='civ.hdmap.Crosswalk.overlap_id', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=122,
  serialized_end=232,
)

_CROSSWALK.fields_by_name['id'].message_type = message_dot_map_dot_proto_dot_map__id__pb2._ID
_CROSSWALK.fields_by_name['polygon'].message_type = message_dot_map_dot_proto_dot_map__geometry__pb2._POLYGON
_CROSSWALK.fields_by_name['overlap_id'].message_type = message_dot_map_dot_proto_dot_map__id__pb2._ID
DESCRIPTOR.message_types_by_name['Crosswalk'] = _CROSSWALK
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Crosswalk = _reflection.GeneratedProtocolMessageType('Crosswalk', (_message.Message,), {
  'DESCRIPTOR' : _CROSSWALK,
  '__module__' : 'message.map.proto.map_crosswalk_pb2'
  # @@protoc_insertion_point(class_scope:civ.hdmap.Crosswalk)
  })
_sym_db.RegisterMessage(Crosswalk)


# @@protoc_insertion_point(module_scope)
