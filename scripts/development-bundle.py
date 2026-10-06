#!/usr/bin/env python3
"""Package a reproducible development artifact without claiming flight qualification."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
ROOT=Path(__file__).resolve().parents[1]
def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--artifact',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    artifact=args.artifact.resolve(strict=True)
    manifest=json.loads(subprocess.check_output([sys.executable,str(ROOT/'scripts/release-manifest.py'),str(artifact),'--profile','remaining-phases-development'],cwd=ROOT))
    args.output.mkdir(parents=True,exist_ok=False)
    files=[artifact,ROOT/'Cargo.lock',ROOT/'apps/esp32/firmware/Cargo.lock',ROOT/'apps/proxy/firmware/Cargo.lock',ROOT/'protocol/common-subset.json',ROOT/'schema/parameters.json',ROOT/'fixtures/hardware/bench-workload.json',ROOT/'fixtures/hardware/esp32s3-mpu6050.json',ROOT/'docs/rust-rewrite/execution-plan.md',ROOT/'docs/rust-rewrite/qualification.md']
    files.extend(ROOT/name for name in ('docs/rust-rewrite/reset-metrics-integration.md','docs/rust-rewrite/proxy-integration.md','apps/proxy/README.md',
        'docs/rust-rewrite/development-runbook.md','docs/rust-rewrite/protocol.md',
        'docs/rust-rewrite/adr-002-wired-session.md','adapters/gazebo/README.md'))
    hashes={}
    for source in files:
        name=str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) and source!=artifact else 'artifact/'+source.name
        target=args.output/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
        hashes[name]=hashlib.sha256(target.read_bytes()).hexdigest()
    manifest.update(bundle_files=hashes,flight_qualified=False,hardware_profile='awaiting-inspection')
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Development bundle saved:',args.output)
if __name__=='__main__':main()
