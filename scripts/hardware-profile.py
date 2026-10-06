#!/usr/bin/env python3
"""Reject incomplete/conflicting hardware input records. Does not qualify hardware."""
import argparse
import json
import math
from pathlib import Path

S3_PINS=set(range(0,22))|set(range(26,49))
def validate(profile):
    errors=[]
    if not isinstance(profile,dict):return ['profile must be an object']
    required_keys={'version','profile','chip','module_marking','board_schematic','flash_mb','psram_mb','reserved_pins','pins','imu','power','airframe','partition_bytes','image_bytes','scope'}
    for key in required_keys-profile.keys():errors.append(key+': missing field')
    for key in ('pins','imu','power','airframe'):
        if not isinstance(profile.get(key),dict):errors.append(key+': object required')
    if errors:return sorted(errors)
    sections={
        'pins': ('sda','scl','rl','rr','fr','fl','enable','battery_adc'),
        'imu': ('identity','axis_rotation_radians','bus_voltage','pullups_ohms','readback_trace'),
        'power': ('driver_part','enable_active_high','reset_pulldown','logic_regulator','battery_cells','battery_discharge_a','loaded_motor_a','divider_ratio'),
        'airframe': ('mass_g','motors','props','rotation_order'),
    }
    for section,keys in sections.items():
        for key in keys:
            if key not in profile[section]:errors.append(section+'.'+key+': missing field')
    def required(value,path):
        if value is None or value=='': errors.append(path+': missing inspected value')
    def walk(value,path=''):
        if isinstance(value,dict):
            for key,item in value.items():walk(item,(path+'.' if path else '')+key)
        else:required(value,path)
    walk(profile)
    if profile.get('version')!=1 or profile.get('chip')!='esp32s3':errors.append('unsupported version/chip')
    if profile.get('scope')!='unqualified-development':errors.append('input record cannot declare flight qualification')
    pins=profile.get('pins',{}); seen={}; reserved=profile.get('reserved_pins')
    for name in ('sda','scl','rl','rr','fr','fl','enable','battery_adc'):
        pin=pins.get(name)
        if type(pin) is not int or pin not in S3_PINS:errors.append('pins.'+name+': invalid S3 GPIO');continue
        if pin in seen:errors.append('pins.'+name+': overlaps '+seen[pin])
        seen[pin]=name
        if isinstance(reserved,list) and pin in reserved:errors.append('pins.'+name+': reserved by board/module')
        if pin in (0,3,45,46):errors.append('pins.'+name+': boot-strapping pin needs a separate reviewed profile')
        if pin in (19,20,43,44):errors.append('pins.'+name+': conflicts with this profile USB/UART console')
        if name=='battery_adc' and pin not in range(1,11):errors.append('battery ADC must use ADC1 GPIO1–10 in this profile')
    if not isinstance(reserved,list):errors.append('reserved_pins: inspected list required')
    power=profile.get('power',{}); imu=profile.get('imu',{})
    if power.get('enable_active_high') is not True or power.get('reset_pulldown') is not True:errors.append('independent active-high enable/reset pulldown required')
    if imu.get('identity') not in (104,105):errors.append('IMU identity must be measured MPU6050 value')
    if imu.get('bus_voltage')!=3.3:errors.append('this profile requires measured 3.3 V I2C logic')
    if power.get('battery_cells')!=1:errors.append('this brushed profile requires 1S; another profile needs review')
    for name in ('battery_discharge_a','loaded_motor_a','divider_ratio'):
        v=power.get(name)
        if type(v) not in (int,float) or not math.isfinite(v) or v<=0:errors.append('power.'+name+': positive measured value required')
    current=power.get('loaded_motor_a');rating=power.get('battery_discharge_a')
    if isinstance(current,(int,float)) and isinstance(rating,(int,float)) and rating<=4*current:errors.append('battery discharge has no margin over four loaded motors')
    partition=profile.get('partition_bytes'); image=profile.get('image_bytes');flash=profile.get('flash_mb')
    if not all(type(x) is int and x>0 for x in (partition,image,flash)):errors.append('positive image/partition/flash sizes required')
    elif image>partition or partition>flash*1024*1024:errors.append('image/partition does not fit flash')
    for key in ('flash_mb','psram_mb'):
        value=profile.get(key)
        if type(value) is not int or value<0:errors.append(key+': nonnegative integer required')
    rotation_imu=imu.get('axis_rotation_radians')
    if not isinstance(rotation_imu,list) or len(rotation_imu)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in rotation_imu):errors.append('IMU rotation requires three finite radians')
    mass=profile.get('airframe',{}).get('mass_g')
    if type(mass) not in (int,float) or not math.isfinite(mass) or mass<=0:errors.append('airframe mass must be positive and measured')
    ratio=power.get('divider_ratio')
    if type(ratio) in (int,float) and ratio>=1:errors.append('divider ratio must be below one')
    rotation=profile.get('airframe',{}).get('rotation_order')
    if rotation not in (['CW','CCW','CW','CCW'],['CCW','CW','CCW','CW']):errors.append('RL RR FR FL must have alternating verified rotation')
    return sorted(set(errors))

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('profile',type=Path);args=parser.parse_args()
    errors=validate(json.loads(args.profile.read_text()))
    print(json.dumps({'status':'incomplete-or-invalid' if errors else 'complete-input-record','hardware_qualified':False,'errors':errors},indent=2))
    return bool(errors)
if __name__=='__main__':raise SystemExit(main())
