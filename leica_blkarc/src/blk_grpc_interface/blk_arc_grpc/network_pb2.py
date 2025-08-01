# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: blk_arc_grpc/network.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='blk_arc_grpc/network.proto',
  package='api.blk',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x1a\x62lk_arc_grpc/network.proto\x12\x07\x61pi.blk\x1a\x1bgoogle/protobuf/empty.proto\"/\n\x19OverrideUSBGatewayRequest\x12\x12\n\nip_address\x18\x01 \x01(\t\"\xd3\x01\n\x1aNetworkInformationResponse\x12\x41\n\ninterfaces\x18\x01 \x03(\x0b\x32-.api.blk.NetworkInformationResponse.Interface\x1ar\n\tInterface\x12\x34\n\x11network_interface\x18\x01 \x01(\x0e\x32\x19.api.blk.NetworkInterface\x12\x0c\n\x04ipv4\x18\x02 \x03(\t\x12\x0c\n\x04ipv6\x18\x03 \x03(\t\x12\x13\n\x0bmac_address\x18\x04 \x01(\t*9\n\x10NetworkInterface\x12\x07\n\x03USB\x10\x00\x12\x0b\n\x07WIFI_AP\x10\x01\x12\x0f\n\x0bWIFI_CLIENT\x10\x02\x32\xb1\x01\n\x07Network\x12T\n\x15GetNetworkInformation\x12\x16.google.protobuf.Empty\x1a#.api.blk.NetworkInformationResponse\x12P\n\x12OverrideUSBGateway\x12\".api.blk.OverrideUSBGatewayRequest\x1a\x16.google.protobuf.Emptyb\x06proto3'
  ,
  dependencies=[google_dot_protobuf_dot_empty__pb2.DESCRIPTOR,])

_NETWORKINTERFACE = _descriptor.EnumDescriptor(
  name='NetworkInterface',
  full_name='api.blk.NetworkInterface',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='USB', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WIFI_AP', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='WIFI_CLIENT', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=331,
  serialized_end=388,
)
_sym_db.RegisterEnumDescriptor(_NETWORKINTERFACE)

NetworkInterface = enum_type_wrapper.EnumTypeWrapper(_NETWORKINTERFACE)
USB = 0
WIFI_AP = 1
WIFI_CLIENT = 2



_OVERRIDEUSBGATEWAYREQUEST = _descriptor.Descriptor(
  name='OverrideUSBGatewayRequest',
  full_name='api.blk.OverrideUSBGatewayRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='ip_address', full_name='api.blk.OverrideUSBGatewayRequest.ip_address', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
  serialized_end=115,
)


_NETWORKINFORMATIONRESPONSE_INTERFACE = _descriptor.Descriptor(
  name='Interface',
  full_name='api.blk.NetworkInformationResponse.Interface',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='network_interface', full_name='api.blk.NetworkInformationResponse.Interface.network_interface', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='ipv4', full_name='api.blk.NetworkInformationResponse.Interface.ipv4', index=1,
      number=2, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='ipv6', full_name='api.blk.NetworkInformationResponse.Interface.ipv6', index=2,
      number=3, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='mac_address', full_name='api.blk.NetworkInformationResponse.Interface.mac_address', index=3,
      number=4, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=b"".decode('utf-8'),
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
  serialized_start=215,
  serialized_end=329,
)

_NETWORKINFORMATIONRESPONSE = _descriptor.Descriptor(
  name='NetworkInformationResponse',
  full_name='api.blk.NetworkInformationResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='interfaces', full_name='api.blk.NetworkInformationResponse.interfaces', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[_NETWORKINFORMATIONRESPONSE_INTERFACE, ],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=118,
  serialized_end=329,
)

_NETWORKINFORMATIONRESPONSE_INTERFACE.fields_by_name['network_interface'].enum_type = _NETWORKINTERFACE
_NETWORKINFORMATIONRESPONSE_INTERFACE.containing_type = _NETWORKINFORMATIONRESPONSE
_NETWORKINFORMATIONRESPONSE.fields_by_name['interfaces'].message_type = _NETWORKINFORMATIONRESPONSE_INTERFACE
DESCRIPTOR.message_types_by_name['OverrideUSBGatewayRequest'] = _OVERRIDEUSBGATEWAYREQUEST
DESCRIPTOR.message_types_by_name['NetworkInformationResponse'] = _NETWORKINFORMATIONRESPONSE
DESCRIPTOR.enum_types_by_name['NetworkInterface'] = _NETWORKINTERFACE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

OverrideUSBGatewayRequest = _reflection.GeneratedProtocolMessageType('OverrideUSBGatewayRequest', (_message.Message,), {
  'DESCRIPTOR' : _OVERRIDEUSBGATEWAYREQUEST,
  '__module__' : 'blk_arc_grpc.network_pb2'
  # @@protoc_insertion_point(class_scope:api.blk.OverrideUSBGatewayRequest)
  })
_sym_db.RegisterMessage(OverrideUSBGatewayRequest)

NetworkInformationResponse = _reflection.GeneratedProtocolMessageType('NetworkInformationResponse', (_message.Message,), {

  'Interface' : _reflection.GeneratedProtocolMessageType('Interface', (_message.Message,), {
    'DESCRIPTOR' : _NETWORKINFORMATIONRESPONSE_INTERFACE,
    '__module__' : 'blk_arc_grpc.network_pb2'
    # @@protoc_insertion_point(class_scope:api.blk.NetworkInformationResponse.Interface)
    })
  ,
  'DESCRIPTOR' : _NETWORKINFORMATIONRESPONSE,
  '__module__' : 'blk_arc_grpc.network_pb2'
  # @@protoc_insertion_point(class_scope:api.blk.NetworkInformationResponse)
  })
_sym_db.RegisterMessage(NetworkInformationResponse)
_sym_db.RegisterMessage(NetworkInformationResponse.Interface)



_NETWORK = _descriptor.ServiceDescriptor(
  name='Network',
  full_name='api.blk.Network',
  file=DESCRIPTOR,
  index=0,
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_start=391,
  serialized_end=568,
  methods=[
  _descriptor.MethodDescriptor(
    name='GetNetworkInformation',
    full_name='api.blk.Network.GetNetworkInformation',
    index=0,
    containing_service=None,
    input_type=google_dot_protobuf_dot_empty__pb2._EMPTY,
    output_type=_NETWORKINFORMATIONRESPONSE,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
  _descriptor.MethodDescriptor(
    name='OverrideUSBGateway',
    full_name='api.blk.Network.OverrideUSBGateway',
    index=1,
    containing_service=None,
    input_type=_OVERRIDEUSBGATEWAYREQUEST,
    output_type=google_dot_protobuf_dot_empty__pb2._EMPTY,
    serialized_options=None,
    create_key=_descriptor._internal_create_key,
  ),
])
_sym_db.RegisterServiceDescriptor(_NETWORK)

DESCRIPTOR.services_by_name['Network'] = _NETWORK

# @@protoc_insertion_point(module_scope)
