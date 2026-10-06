#!/usr/bin/env python3
"""Collect a frozen stopped-owner snapshot over direct UART; no qualification claim."""
import argparse
import json
from pathlib import Path
import time
from pyflix.client import Flix
from pyflix.serial import SerialTransport

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    # Exclusive creation protects earlier evidence and verifies the path first.
    with args.output.open('x') as output:
        with Flix(SerialTransport(args.device)) as client:
            client.disarm();client.console('maintenance')
            deadline=time.monotonic()+2
            while client.status()!='maintenance':
                if time.monotonic()>=deadline:raise RuntimeError('maintenance unavailable')
            result=client.metrics()
            result.update(status='observed-development-snapshot',device=args.device,
                          physical_stop_verified=False,workload_verified=False)
            json.dump(result,output,indent=2);output.write('\n');output.flush()
            client.console('exit')
    print('Stopped timing snapshot saved:',args.output)

if __name__=='__main__':main()
