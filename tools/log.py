"""Bounded versioned log download and CSV conversion; no legacy native-memory decoding."""
import csv, io, math, struct
def download(client):
    data=bytearray();total=None
    while total is None or len(data)<total:
        offset=len(data);reply=client.log_chunk(offset,min(90,total-offset) if total else 90)
        received,identifier,count=struct.unpack_from('<IHB',reply)
        if received!=offset or identifier!=0 or count==0 or count>90 or len(reply)<7+count:
            raise ValueError('incomplete or out-of-order log')
        data.extend(reply[7:7+count])
        if total is None:
            if len(data)<12: raise ValueError('missing log header')
            magic,version,size,rows=struct.unpack_from('<4sHHI',data)
            if magic!=b'FLXR' or version!=1 or size!=64 or rows>1000: raise ValueError('log header')
            total=12+rows*size
        if len(data)>total: raise ValueError('log length exceeded')
    return bytes(data)
def to_csv(data):
    if len(data)<12: raise ValueError('truncated log')
    magic,version,size,count=struct.unpack_from('<4sHHI',data)
    if magic!=b'FLXR' or version!=1 or size!=64 or count>1000 or len(data)!=12+count*64: raise ValueError('log size/version')
    stream=io.StringIO();writer=csv.writer(stream)
    writer.writerow(['us','t','gx','gy','gz','gx_target','gy_target','gz_target','roll','pitch','yaw','roll_target','pitch_target','yaw_target','applied_mean_thrust'])
    previous=None
    for i in range(count):
        at,*values=struct.unpack_from('<Q14f',data,12+i*64)
        if (previous is not None and at<=previous) or not all(map(math.isfinite,values)): raise ValueError('invalid log record')
        previous=at;writer.writerow([at,*values])
    return stream.getvalue()
