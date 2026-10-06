#!/usr/bin/env python3
"""Describe artifacts; refuse to label an unqualified build a flight release."""
import argparse,hashlib,json,pathlib,subprocess,os,shutil
ROOT=pathlib.Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('artifact',type=pathlib.Path);p.add_argument('--profile',required=True);args=p.parse_args()
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def command(*args):return subprocess.run(args,cwd=ROOT,text=True,capture_output=True,check=True).stdout.strip()
artifact=args.artifact.resolve(strict=True)
# Fingerprint local code even before the first commit; omit documentation/evidence
# so writing this manifest cannot change the next source fingerprint.
git_available=shutil.which('git') is not None
if git_available:
    source_paths=sorted(set(command('git','ls-files','--cached','--others','--exclude-standard').splitlines()))
else:
    # Cached simulator images may predate Git installation. Record a bounded
    # code inventory rather than silently omitting source provenance.
    source_paths=[]
    for folder,dirs,files in os.walk(ROOT):
        dirs[:]=[d for d in dirs if d not in ('.git','.local','target','build','__pycache__','docs')]
        for name in files:
            if name.startswith('.env') or name in ('sdkconfig','sdkconfig.old','.DS_Store'):continue
            if name.endswith(('.pyc','.log','.bin','.elf','.a','.so','.dylib','.session.json','.enrollment.frame','.proxy.json')):continue
            path=pathlib.Path(folder)/name
            if path.is_file() and not path.is_symlink():source_paths.append(str(path.relative_to(ROOT)))
    source_paths=sorted(source_paths)
source_hashes = {name: sha(ROOT/name) for name in source_paths
                 if not name.startswith('docs/') and (ROOT/name).is_file()}
source_digest = hashlib.sha256(json.dumps(source_hashes,sort_keys=True,separators=(',',':')).encode()).hexdigest()

manifest={'status':'unqualified-development','profile':args.profile,'artifact':artifact.name,'sha256':sha(artifact),'source_revision':command('git','rev-parse','HEAD') if git_available and (ROOT/'.git/HEAD').exists() and subprocess.run(['git','rev-parse','--verify','HEAD'],cwd=ROOT,capture_output=True).returncode==0 else 'uncommitted','dirty':bool(command('git','status','--porcelain')) if git_available else True,'rustc':command('rustc','--version'),'cargo_lock_sha256':sha(ROOT/'Cargo.lock'),'schema_sha256':sha(ROOT/'schema/parameters.json'),'dialect_sha256':sha(ROOT/'protocol/common-subset.json'),'target_sdk':'esp-idf-v5.4.2 (only for board artifacts)','hardware_evidence':None}
manifest.update(source_inventory_method='git' if git_available else 'code-filewalk', source_snapshot_sha256=source_digest, source_files=source_hashes,
                firmware_lock_sha256=sha(ROOT/'apps/esp32/firmware/Cargo.lock'),
                proxy_firmware_lock_sha256=sha(ROOT/'apps/proxy/firmware/Cargo.lock'))
print(json.dumps(manifest,indent=2))
