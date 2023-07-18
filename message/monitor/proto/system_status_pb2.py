# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: message/monitor/proto/system_status.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from message.common.proto import header_pb2 as message_dot_common_dot_proto_dot_header__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='message/monitor/proto/system_status.proto',
  package='civ.monitor',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n)message/monitor/proto/system_status.proto\x12\x0b\x63iv.monitor\x1a!message/common/proto/header.proto\"\x9f\x01\n\x0f\x43omponentStatus\x12<\n\x06status\x18\x01 \x01(\x0e\x32#.civ.monitor.ComponentStatus.Status:\x07UNKNOWN\x12\x0f\n\x07message\x18\x02 \x01(\t\"=\n\x06Status\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x06\n\x02OK\x10\x01\x12\x08\n\x04WARN\x10\x02\x12\t\n\x05\x45RROR\x10\x03\x12\t\n\x05\x46\x41TAL\x10\x04\"\x91\x02\n\tComponent\x12-\n\x07summary\x18\x01 \x01(\x0b\x32\x1c.civ.monitor.ComponentStatus\x12\x34\n\x0eprocess_status\x18\x02 \x01(\x0b\x32\x1c.civ.monitor.ComponentStatus\x12\x34\n\x0e\x63hannel_status\x18\x03 \x01(\x0b\x32\x1c.civ.monitor.ComponentStatus\x12\x35\n\x0fresource_status\x18\x04 \x01(\x0b\x32\x1c.civ.monitor.ComponentStatus\x12\x32\n\x0cother_status\x18\x05 \x01(\x0b\x32\x1c.civ.monitor.ComponentStatus\"\xd5\x03\n\x0cSystemStatus\x12\"\n\x06header\x18\x01 \x01(\x0b\x32\x12.civ.common.Header\x12>\n\x0bhmi_modules\x18\x07 \x03(\x0b\x32).civ.monitor.SystemStatus.HmiModulesEntry\x12=\n\ncomponents\x18\x08 \x03(\x0b\x32).civ.monitor.SystemStatus.ComponentsEntry\x12\x15\n\rpassenger_msg\x18\x04 \x01(\t\x12 \n\x18safety_mode_trigger_time\x18\x05 \x01(\x01\x12\x1e\n\x16require_emergency_stop\x18\x06 \x01(\x08\x12!\n\x19is_realtime_in_simulation\x18\t \x01(\x08\x1aO\n\x0fHmiModulesEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12+\n\x05value\x18\x02 \x01(\x0b\x32\x1c.civ.monitor.ComponentStatus:\x02\x38\x01\x1aI\n\x0f\x43omponentsEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12%\n\x05value\x18\x02 \x01(\x0b\x32\x16.civ.monitor.Component:\x02\x38\x01J\x04\x08\x02\x10\x03J\x04\x08\x03\x10\x04'
  ,
  dependencies=[message_dot_common_dot_proto_dot_header__pb2.DESCRIPTOR,])



_COMPONENTSTATUS_STATUS = _descriptor.EnumDescriptor(
  name='Status',
  full_name='civ.monitor.ComponentStatus.Status',
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
      name='OK', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WARN', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='ERROR', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='FATAL', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=192,
  serialized_end=253,
)
_sym_db.RegisterEnumDescriptor(_COMPONENTSTATUS_STATUS)


_COMPONENTSTATUS = _descriptor.Descriptor(
  name='ComponentStatus',
  full_name='civ.monitor.ComponentStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='status', full_name='civ.monitor.ComponentStatus.status', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=True, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='message', full_name='civ.monitor.ComponentStatus.message', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _COMPONENTSTATUS_STATUS,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=94,
  serialized_end=253,
)


_COMPONENT = _descriptor.Descriptor(
  name='Component',
  full_name='civ.monitor.Component',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='summary', full_name='civ.monitor.Component.summary', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='process_status', full_name='civ.monitor.Component.process_status', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='channel_status', full_name='civ.monitor.Component.channel_status', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='resource_status', full_name='civ.monitor.Component.resource_status', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='other_status', full_name='civ.monitor.Component.other_status', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
  serialized_start=256,
  serialized_end=529,
)


_SYSTEMSTATUS_HMIMODULESENTRY = _descriptor.Descriptor(
  name='HmiModulesEntry',
  full_name='civ.monitor.SystemStatus.HmiModulesEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='civ.monitor.SystemStatus.HmiModulesEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='civ.monitor.SystemStatus.HmiModulesEntry.value', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=835,
  serialized_end=914,
)

_SYSTEMSTATUS_COMPONENTSENTRY = _descriptor.Descriptor(
  name='ComponentsEntry',
  full_name='civ.monitor.SystemStatus.ComponentsEntry',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='key', full_name='civ.monitor.SystemStatus.ComponentsEntry.key', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='value', full_name='civ.monitor.SystemStatus.ComponentsEntry.value', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=b'8\001',
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=916,
  serialized_end=989,
)

_SYSTEMSTATUS = _descriptor.Descriptor(
  name='SystemStatus',
  full_name='civ.monitor.SystemStatus',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='header', full_name='civ.monitor.SystemStatus.header', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='hmi_modules', full_name='civ.monitor.SystemStatus.hmi_modules', index=1,
      number=7, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='components', full_name='civ.monitor.SystemStatus.components', index=2,
      number=8, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='passenger_msg', full_name='civ.monitor.SystemStatus.passenger_msg', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='safety_mode_trigger_time', full_name='civ.monitor.SystemStatus.safety_mode_trigger_time', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='require_emergency_stop', full_name='civ.monitor.SystemStatus.require_emergency_stop', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='is_realtime_in_simulation', full_name='civ.monitor.SystemStatus.is_realtime_in_simulation', index=6,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_SYSTEMSTATUS_HMIMODULESENTRY, _SYSTEMSTATUS_COMPONENTSENTRY, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=532,
  serialized_end=1001,
)

_COMPONENTSTATUS.fields_by_name['status'].enum_type = _COMPONENTSTATUS_STATUS
_COMPONENTSTATUS_STATUS.containing_type = _COMPONENTSTATUS
_COMPONENT.fields_by_name['summary'].message_type = _COMPONENTSTATUS
_COMPONENT.fields_by_name['process_status'].message_type = _COMPONENTSTATUS
_COMPONENT.fields_by_name['channel_status'].message_type = _COMPONENTSTATUS
_COMPONENT.fields_by_name['resource_status'].message_type = _COMPONENTSTATUS
_COMPONENT.fields_by_name['other_status'].message_type = _COMPONENTSTATUS
_SYSTEMSTATUS_HMIMODULESENTRY.fields_by_name['value'].message_type = _COMPONENTSTATUS
_SYSTEMSTATUS_HMIMODULESENTRY.containing_type = _SYSTEMSTATUS
_SYSTEMSTATUS_COMPONENTSENTRY.fields_by_name['value'].message_type = _COMPONENT
_SYSTEMSTATUS_COMPONENTSENTRY.containing_type = _SYSTEMSTATUS
_SYSTEMSTATUS.fields_by_name['header'].message_type = message_dot_common_dot_proto_dot_header__pb2._HEADER
_SYSTEMSTATUS.fields_by_name['hmi_modules'].message_type = _SYSTEMSTATUS_HMIMODULESENTRY
_SYSTEMSTATUS.fields_by_name['components'].message_type = _SYSTEMSTATUS_COMPONENTSENTRY
DESCRIPTOR.message_types_by_name['ComponentStatus'] = _COMPONENTSTATUS
DESCRIPTOR.message_types_by_name['Component'] = _COMPONENT
DESCRIPTOR.message_types_by_name['SystemStatus'] = _SYSTEMSTATUS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

ComponentStatus = _reflection.GeneratedProtocolMessageType('ComponentStatus', (_message.Message,), {
  'DESCRIPTOR' : _COMPONENTSTATUS,
  '__module__' : 'message.monitor.proto.system_status_pb2'
  # @@protoc_insertion_point(class_scope:civ.monitor.ComponentStatus)
  })
_sym_db.RegisterMessage(ComponentStatus)

Component = _reflection.GeneratedProtocolMessageType('Component', (_message.Message,), {
  'DESCRIPTOR' : _COMPONENT,
  '__module__' : 'message.monitor.proto.system_status_pb2'
  # @@protoc_insertion_point(class_scope:civ.monitor.Component)
  })
_sym_db.RegisterMessage(Component)

SystemStatus = _reflection.GeneratedProtocolMessageType('SystemStatus', (_message.Message,), {

  'HmiModulesEntry' : _reflection.GeneratedProtocolMessageType('HmiModulesEntry', (_message.Message,), {
    'DESCRIPTOR' : _SYSTEMSTATUS_HMIMODULESENTRY,
    '__module__' : 'message.monitor.proto.system_status_pb2'
    # @@protoc_insertion_point(class_scope:civ.monitor.SystemStatus.HmiModulesEntry)
    })
  ,

  'ComponentsEntry' : _reflection.GeneratedProtocolMessageType('ComponentsEntry', (_message.Message,), {
    'DESCRIPTOR' : _SYSTEMSTATUS_COMPONENTSENTRY,
    '__module__' : 'message.monitor.proto.system_status_pb2'
    # @@protoc_insertion_point(class_scope:civ.monitor.SystemStatus.ComponentsEntry)
    })
  ,
  'DESCRIPTOR' : _SYSTEMSTATUS,
  '__module__' : 'message.monitor.proto.system_status_pb2'
  # @@protoc_insertion_point(class_scope:civ.monitor.SystemStatus)
  })
_sym_db.RegisterMessage(SystemStatus)
_sym_db.RegisterMessage(SystemStatus.HmiModulesEntry)
_sym_db.RegisterMessage(SystemStatus.ComponentsEntry)


_SYSTEMSTATUS_HMIMODULESENTRY._options = None
_SYSTEMSTATUS_COMPONENTSENTRY._options = None
# @@protoc_insertion_point(module_scope)
