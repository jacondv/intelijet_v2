# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: blk_arc_grpc/storage.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='blk_arc_grpc/storage.proto',
  package='api.blk',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x1a\x62lk_arc_grpc/storage.proto\x12\x07\x61pi.blk\x1a\x1bgoogle/protobuf/empty.proto\"5\n\x12\x44iskStatusResponse\x12\x1f\n\x17\x66ree_disk_space_percent\x18\x01 \x01(\x02\x32O\n\x07Storage\x12\x44\n\rGetDiskStatus\x12\x16.google.protobuf.Empty\x1a\x1b.api.blk.DiskStatusResponseb\x06proto3'
  ,
  dependencies=[google_dot_protobuf_dot_empty__pb2.DESCRIPTOR,])




_DISKSTATUSRESPONSE = _descriptor.Descriptor(
  name='DiskStatusResponse',
  full_name='api.blk.DiskStatusResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='free_disk_space_percent', full_name='api.blk.DiskStatusResponse.free_disk_space_percent', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=68,
  serialized_end=121,
)

DESCRIPTOR.message_types_by_name['DiskStatusResponse'] = _DISKSTATUSRESPONSE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DiskStatusResponse = _reflection.GeneratedProtocolMessageType('DiskStatusResponse', (_message.Message,), {
  'DESCRIPTOR' : _DISKSTATUSRESPONSE,
  '__module__' : 'blk_arc_grpc.storage_pb2'
  # @@protoc_insertion_point(class_scope:api.blk.DiskStatusResponse)
  })
_sym_db.RegisterMessage(DiskStatusResponse)



_STORAGE = _descriptor.ServiceDescriptor(
  name='Storage',
  full_name='api.blk.Storage',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=123,
  serialized_end=202,
  methods=[
  _descriptor.MethodDescriptor(
    name='GetDiskStatus',
    full_name='api.blk.Storage.GetDiskStatus',
    index=0,
    containing_service=None,
    input_type=google_dot_protobuf_dot_empty__pb2._EMPTY,
    output_type=_DISKSTATUSRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_STORAGE)

DESCRIPTOR.services_by_name['Storage'] = _STORAGE

# @@protoc_insertion_point(module_scope)
