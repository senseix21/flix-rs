#!/usr/bin/env python3
"""Instrument a disposable pinned checkout; never mutate the legacy repository."""
import argparse
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('checkout',type=Path);args=parser.parse_args()
root=args.checkout
plugin=root/'gazebo/simulator.cpp';text=plugin.read_text()
# Retain the actual legacy filter/controller/tick order. Disable only the hidden
# yaw truth injection, to compare the same uncorrected estimator profile.
assert 'attitude.setYaw(this->model->WorldPose().Yaw());' in text
text=text.replace('attitude.setYaw(this->model->WorldPose().Yaw());','// Comparison profile: no hidden truth-yaw correction.')
text=text.replace('using ignition::math::Vector3d;', '''
namespace {
int trace_fd = -1;
sockaddr_in trace_endpoint{};
uint64_t trace_sequence = 0;
void put32(uint8_t *p, uint32_t v) { for(int i=0;i<4;i++)p[i]=v>>(8*i); }
void put64(uint8_t *p, uint64_t v) { for(int i=0;i<8;i++)p[i]=v>>(8*i); }
void legacy_trace_command(const uint8_t *frame, unsigned length) {
  if(trace_fd<0 || length>280)return;
  uint8_t p[304]{};memcpy(p,"FLXC",4);put32(p+4,1);put64(p+8,__micros);
  put32(p+16,1);put32(p+20,length);memcpy(p+24,frame,length);
  sendto(trace_fd,p,24+length,MSG_DONTWAIT,(sockaddr*)&trace_endpoint,sizeof(trace_endpoint));
}
}
using ignition::math::Vector3d;''')
# Declaration must precede mavlink.ino; implementation comes after includes.
text=text.replace('#include "mavlink.ino"','void legacy_trace_command(const uint8_t *, unsigned);\n#include "mavlink.ino"')
# Definition must use the same global linkage as the forward declaration.
text=text.replace('namespace {\nint trace_fd', 'int trace_fd').replace('\n}\nusing ignition::math::Vector3d;', '\nusing ignition::math::Vector3d;')
text=text.replace('this->model = _parent;', '''this->model = _parent;
        trace_fd = socket(AF_INET, SOCK_DGRAM, 0);
        trace_endpoint.sin_family=AF_INET;
        trace_endpoint.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        trace_endpoint.sin_port=htons(14560);
        udpLocalPort=14550;udpRemotePort=14561;udpRemoteIP="127.0.0.1";''')
text=text.replace('\t\tlogData();','''        traceTick();
        // Comparison instrumentation omits legacy disk log IO.
''')
marker='\tvoid applyMotorForces() {'
assert marker in text
method='''
    void traceTick() {
      uint8_t p[184]{};memcpy(p,"FLXT",4);put32(p+4,1);put64(p+8,__micros);
      put64(p+16,0);put32(p+24,armed?3:1);put32(p+28,0);put32(p+32,0);
      put32(p+36,1);put64(p+40,trace_sequence++);
      const auto pose=model->WorldPose();const auto velocity=body->WorldLinearVel();
      const float v[34]={
        float(pose.Pos().X()),float(pose.Pos().Y()),float(pose.Pos().Z()),
        float(pose.Rot().W()),float(pose.Rot().X()),float(pose.Rot().Y()),float(pose.Rot().Z()),
        float(velocity.X()),float(velocity.Y()),float(velocity.Z()),
        gyro.x,gyro.y,gyro.z,acc.x,acc.y,acc.z,
        attitude.w,attitude.x,attitude.y,attitude.z,rates.x,rates.y,rates.z,
        ratesTarget.x,ratesTarget.y,ratesTarget.z,
        attitudeTarget.w,attitudeTarget.x,attitudeTarget.y,attitudeTarget.z,
        motors[0],motors[1],motors[2],motors[3]};
      for(int i=0;i<34;i++){uint32_t bits;memcpy(&bits,v+i,4);put32(p+48+4*i,bits);}
      sendto(trace_fd,p,sizeof(p),MSG_DONTWAIT,(sockaddr*)&trace_endpoint,sizeof(trace_endpoint));
    }
'''
text=text.replace(marker,method+marker)
plugin.write_text(text)
p=root/'flix/mavlink.ino';s=p.read_text();needle='const mavlink_message_t& msg = *(mavlink_message_t *)_msg;';assert needle in s
s=s.replace(needle,needle+'\n\tuint8_t frame[MAVLINK_MAX_PACKET_LEN];\n\tlegacy_trace_command(frame, mavlink_msg_to_send_buffer(frame, &msg));');p.write_text(s)

p=root/'gazebo/CMakeLists.txt';s=p.read_text()
s=s.replace('list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")', 'string(REPLACE ";" " " GAZEBO_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")\nset(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")')
p.write_text(s)
