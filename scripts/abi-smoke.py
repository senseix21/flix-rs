#!/usr/bin/env python3
"""Exercise the exported C ABI using an independent ctypes layout."""
import ctypes,pathlib,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
library=ctypes.CDLL(str(ROOT/'target/release'/('libflight_gazebo.dylib' if sys.platform=='darwin' else 'libflight_gazebo.so')))
class Input(ctypes.Structure):_fields_=[('version',ctypes.c_uint32),('calibrated',ctypes.c_uint32),('now',ctypes.c_uint64),('sample',ctypes.c_uint64),('gyro',ctypes.c_float*3),('acc',ctypes.c_float*3)]
class Output(ctypes.Structure):_fields_=[('version',ctypes.c_uint32),('state',ctypes.c_uint32),('reason',ctypes.c_uint32),('reserved',ctypes.c_uint32),('now',ctypes.c_uint64),('generation',ctypes.c_uint64),('motors',ctypes.c_float*4)]
library.flix_create.restype=ctypes.c_uint64
library.flix_tick.argtypes=[ctypes.c_uint64,ctypes.POINTER(Input),ctypes.POINTER(Output)]
library.flix_destroy.argtypes=[ctypes.c_uint64]
assert ctypes.sizeof(Input)==ctypes.sizeof(Output)==48
handle=library.flix_create();assert handle
sample=Input(1,1,0,0,(ctypes.c_float*3)(0,0,0),(ctypes.c_float*3)(0,0,9.80665));output=Output()
assert library.flix_tick(handle,ctypes.byref(sample),ctypes.byref(output))==0
assert output.state==1 and list(output.motors)==[0]*4
assert library.flix_destroy(handle)==0
assert library.flix_tick(handle,ctypes.byref(sample),ctypes.byref(output))==-2
assert list(output.motors)==[0]*4
print('C ABI: independent layout, stopped output, stale handle passed')
