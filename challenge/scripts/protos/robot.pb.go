// Code generated by protoc-gen-go. DO NOT EDIT.
// source: robot.proto

/*
Package protos is a generated protocol buffer package.

It is generated from these files:
	robot.proto

It has these top-level messages:
	Llanta
	Request
*/
package protos

import proto "github.com/golang/protobuf/proto"
import fmt "fmt"
import math "math"
import _ "google.golang.org/genproto/googleapis/api/annotations"

// Reference imports to suppress errors if they are not otherwise used.
var _ = proto.Marshal
var _ = fmt.Errorf
var _ = math.Inf

// This is a compile-time assertion to ensure that this generated file
// is compatible with the proto package it is being compiled against.
// A compilation error at this line likely means your copy of the
// proto package needs to be updated.
const _ = proto.ProtoPackageIsVersion2 // please upgrade the proto package

type Llanta struct {
	Wl float32 `protobuf:"fixed32,1,opt,name=wl" json:"wl,omitempty"`
	Wr float32 `protobuf:"fixed32,2,opt,name=wr" json:"wr,omitempty"`
}

func (m *Llanta) Reset()                    { *m = Llanta{} }
func (m *Llanta) String() string            { return proto.CompactTextString(m) }
func (*Llanta) ProtoMessage()               {}
func (*Llanta) Descriptor() ([]byte, []int) { return fileDescriptor0, []int{0} }

func (m *Llanta) GetWl() float32 {
	if m != nil {
		return m.Wl
	}
	return 0
}

func (m *Llanta) GetWr() float32 {
	if m != nil {
		return m.Wr
	}
	return 0
}

type Request struct {
	Query string `protobuf:"bytes,1,opt,name=query" json:"query,omitempty"`
}

func (m *Request) Reset()                    { *m = Request{} }
func (m *Request) String() string            { return proto.CompactTextString(m) }
func (*Request) ProtoMessage()               {}
func (*Request) Descriptor() ([]byte, []int) { return fileDescriptor0, []int{1} }

func (m *Request) GetQuery() string {
	if m != nil {
		return m.Query
	}
	return ""
}

func init() {
	proto.RegisterType((*Llanta)(nil), "RPCDemoPkg.Llanta")
	proto.RegisterType((*Request)(nil), "RPCDemoPkg.Request")
}

func init() { proto.RegisterFile("robot.proto", fileDescriptor0) }

var fileDescriptor0 = []byte{
	// 231 bytes of a gzipped FileDescriptorProto
	0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xff, 0x4c, 0x8f, 0xb1, 0x4a, 0x03, 0x41,
	0x10, 0x86, 0xb9, 0x05, 0xa3, 0x8e, 0x92, 0x62, 0xb5, 0x08, 0x41, 0x88, 0x1c, 0x16, 0x21, 0xc5,
	0x2d, 0x68, 0x67, 0xa9, 0x96, 0x82, 0x61, 0xed, 0xec, 0xf6, 0x74, 0x5c, 0x0e, 0xef, 0x76, 0x92,
	0xd9, 0x39, 0x8f, 0xb4, 0xbe, 0x82, 0x8f, 0xe6, 0x2b, 0xf8, 0x20, 0xe2, 0xee, 0x81, 0x56, 0xc3,
	0x37, 0x7c, 0xcc, 0xfc, 0x3f, 0x1c, 0x31, 0xd5, 0x24, 0xd5, 0x86, 0x49, 0x48, 0x83, 0x5d, 0xdf,
	0xde, 0x61, 0x47, 0xeb, 0x37, 0x3f, 0x3f, 0xf3, 0x44, 0xbe, 0x45, 0xe3, 0x36, 0x8d, 0x71, 0x21,
	0x90, 0x38, 0x69, 0x28, 0xc4, 0x6c, 0x96, 0x4b, 0x98, 0xdc, 0xb7, 0x2e, 0x88, 0xd3, 0x53, 0x50,
	0x43, 0x3b, 0x2b, 0xce, 0x8b, 0xa5, 0xb2, 0x6a, 0x68, 0x13, 0xf3, 0x4c, 0x8d, 0xcc, 0xe5, 0x02,
	0xf6, 0x2d, 0x6e, 0x7b, 0x8c, 0xa2, 0x4f, 0x61, 0x6f, 0xdb, 0x23, 0xef, 0x92, 0x7d, 0x68, 0x33,
	0x5c, 0xf6, 0x70, 0x6c, 0x7f, 0x33, 0x3c, 0x22, 0xbf, 0x37, 0xcf, 0xa8, 0x11, 0xa6, 0x0f, 0xb5,
	0x60, 0x40, 0xce, 0x1f, 0xa2, 0x3e, 0xa9, 0xfe, 0x72, 0x55, 0xe3, 0xb1, 0xb9, 0xfe, 0xbf, 0xcc,
	0x66, 0xb9, 0xfa, 0xf8, 0xfa, 0xfe, 0x54, 0x17, 0xe5, 0xc2, 0x30, 0x46, 0xf1, 0x4e, 0x70, 0x70,
	0xbb, 0x17, 0xec, 0xc8, 0x78, 0x94, 0x26, 0xbc, 0x12, 0x77, 0xa9, 0xc5, 0x75, 0xb1, 0xba, 0x81,
	0xa7, 0x83, 0xca, 0xa4, 0x32, 0xb1, 0x9e, 0xa4, 0x79, 0xf5, 0x13, 0x00, 0x00, 0xff, 0xff, 0x0f,
	0x65, 0xaa, 0xec, 0x0d, 0x01, 0x00, 0x00,
}
