#!/usr/bin/env python3
"""Create, verify, install and roll back local development packages. Never flash."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import tarfile
import tempfile

ROOT=Path(__file__).resolve().parents[1]

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def member(root,name):
    path=PurePosixPath(name)
    if path.is_absolute() or not path.parts or any(x in ('..','.') for x in path.parts):
        raise ValueError('unsafe package path')
    target=root.joinpath(*path.parts)
    if any(p.is_symlink() for p in [target,*target.parents] if p!=root.parent):
        raise ValueError('package symlink forbidden')
    if not target.resolve().is_relative_to(root.resolve()):raise ValueError('path escapes package')
    return target

def verify(directory):
    directory=directory.resolve(strict=True)
    manifest=json.loads((directory/'manifest.json').read_text())
    if manifest.get('version')!=1 or manifest.get('flight_qualified') is not False:
        raise ValueError('unsupported or incorrectly qualified package')
    files=manifest['files']
    if not files or len(files)>10000:raise ValueError('invalid inventory')
    for name,digest in files.items():
        path=member(directory,name)
        if not path.is_file() or sha(path)!=digest:raise ValueError('package file failed verification: '+name)
    entries=list(directory.rglob('*'))
    if any(p.is_symlink() or not (p.is_file() or p.is_dir()) for p in entries):
        raise ValueError('package contains symlink or special file')
    actual={str(p.relative_to(directory)) for p in entries if p.is_file()}
    if actual!=set(files)|{'manifest.json'}:raise ValueError('unlisted package files')
    for profile in manifest['profiles']:
        base=member(directory,'profiles/'+profile)
        metadata=json.loads((base/'flasher_args.json').read_text())
        spans=[]
        for offset,name in metadata['flash_files'].items():
            path=member(base,name)
            if str(path.relative_to(directory)) not in files:raise ValueError('unhashed flash file')
            start=int(offset,0);end=start+path.stat().st_size
            if start<0 or end>2**24:raise ValueError('invalid flash span')
            if any(start<b and end>a for a,b in spans):raise ValueError('overlapping flash files')
            spans.append((start,end))
    return manifest

def create(destination,builds):
    if destination.exists():raise FileExistsError('package destination exists')
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.package-',dir=destination.parent) as temporary:
        stage=Path(temporary)/'bundle';stage.mkdir()
        profiles=[]
        for specification in builds:
            name,separator,value=specification.partition('=')
            if not separator or not name.replace('-','').isalnum() or name in profiles:
                raise ValueError('use unique PROFILE=SDK_BUILD')
            source=Path(value).resolve(strict=True)
            metadata=json.loads((source/'flasher_args.json').read_text())
            if metadata['extra_esptool_args']['chip']!='esp32s3':raise ValueError('unexpected chip')
            if {int(offset,0) for offset in metadata['flash_files']}!={0,0x8000,0x10000}:
                raise ValueError('only factory app/boot/partition layout is supported; provision NVS separately')
            config=(source/'sdkconfig').read_text()
            for line in config.splitlines():
                if 'PASSWORD=' in line and line.split('=',1)[1] not in ('""',''):
                    raise ValueError('refusing to package network credentials')
            names=set(metadata['flash_files'].values())|{'sdkconfig','flasher_args.json','flash_args'}
            for item in names:
                src=member(source,item);dst=member(stage,'profiles/'+name+'/'+item)
                dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
            profiles.append(name)
        # Git's ignore rules exclude caches, private provisioning, SDKs and binaries.
        names=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard'],cwd=ROOT,text=True).splitlines()
        snapshot={}
        with tarfile.open(stage/'source.tar.gz','w:gz') as archive:
            for name in sorted(set(names)):
                source=member(ROOT,name)
                if not source.is_file():continue
                if name.endswith(('.session.json','.enrollment.frame','.proxy.json','.nvs.bin')):
                    raise ValueError('private file entered source inventory')
                snapshot[name]=sha(source)
                info=archive.gettarinfo(str(source),arcname=name)
                info.uid=info.gid=0;info.uname=info.gname='';info.mtime=0
                with source.open('rb') as stream:archive.addfile(info,stream)
        (stage/'source-files.json').write_text(json.dumps(snapshot,indent=2)+'\n')
        (stage/'INSTALL.md').write_text('''# Development installation and recovery

Run `python3 scripts/software-package.py verify PACKAGE` before use.
Run `install PACKAGE DESTINATION` to stage a local verified copy atomically.
A replacement keeps DESTINATION.previous. `rollback DESTINATION` swaps verified copies.
These operations do not change a board. Choose the inspected hardware profile and
follow docs/rust-rewrite/development-runbook.md before any explicit flashing.
Each profiles/ directory contains app, bootloader, partition table, SDK config,
flash_args and flasher_args.json. Run esptool only from the chosen profile directory.
Private peer credentials and fresh per-boot MAVLink keys are provisioned separately.
Never flash a profile with enabled motors before verifying its pins and driver polarity.
No hardware upload, radio qualification, measured stop latency, or flight is claimed.
''')
        inventory={str(p.relative_to(stage)):sha(p) for p in stage.rglob('*') if p.is_file()}
        manifest={'version':1,'flight_qualified':False,'status':'development-only','profiles':profiles,
                  'files':inventory,'sdk':'ESP-IDF v5.4.2','rustc':subprocess.check_output(['rustc','--version'],text=True).strip()}
        (stage/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        verify(stage)
        os.replace(stage,destination)

def install(source,destination):
    verify(source)
    destination=destination.absolute();previous=destination.with_name(destination.name+'.previous')
    if destination.is_symlink() or previous.is_symlink():raise ValueError('installation symlink forbidden')
    if previous.exists():raise FileExistsError('previous package exists; preserve it before replacing again')
    if destination.exists():verify(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.install-',dir=destination.parent) as temporary:
        stage=Path(temporary)/'bundle';shutil.copytree(source,stage,symlinks=True);verify(stage)
        if destination.exists():os.replace(destination,previous)
        try:os.replace(stage,destination)
        except BaseException:
            if previous.exists() and not destination.exists():os.replace(previous,destination)
            raise

def extract(source,destination):
    verify(source)
    if destination.exists():raise FileExistsError('source destination exists')
    destination.parent.mkdir(parents=True,exist_ok=True)
    inventory=json.loads((source/'source-files.json').read_text())
    with tempfile.TemporaryDirectory(prefix='.source-',dir=destination.parent) as temporary:
        stage=Path(temporary)/'source';stage.mkdir()
        with tarfile.open(source/'source.tar.gz','r:gz') as archive:
            members=archive.getmembers()
            if len(members)!=len(inventory):raise ValueError('source archive inventory mismatch')
            for entry in members:
                if not entry.isfile() or entry.name not in inventory:
                    raise ValueError('unexpected source archive member')
                member(stage,entry.name)
            archive.extractall(stage,filter='data')
        for name,digest in inventory.items():
            if sha(member(stage,name))!=digest:raise ValueError('source hash mismatch')
        os.replace(stage,destination)

def rollback(destination):
    destination=destination.absolute();previous=destination.with_name(destination.name+'.previous')
    if destination.is_symlink() or previous.is_symlink():raise ValueError('installation symlink forbidden')
    verify(destination);verify(previous)
    # A power interruption may leave .swap; never overwrite that recovery copy.
    temporary=destination.with_name(destination.name+'.swap')
    if temporary.exists():raise FileExistsError('rollback recovery copy exists')
    os.replace(destination,temporary)
    try:
        os.replace(previous,destination)
        os.replace(temporary,previous)
    except BaseException:
        if not destination.exists():os.replace(temporary,destination)
        raise

def main():
    parser=argparse.ArgumentParser(description=__doc__);commands=parser.add_subparsers(dest='command',required=True)
    p=commands.add_parser('create');p.add_argument('destination',type=Path);p.add_argument('--build',action='append',required=True)
    p=commands.add_parser('verify');p.add_argument('directory',type=Path)
    p=commands.add_parser('install');p.add_argument('source',type=Path);p.add_argument('destination',type=Path)
    p=commands.add_parser('extract');p.add_argument('source',type=Path);p.add_argument('destination',type=Path)
    p=commands.add_parser('rollback');p.add_argument('destination',type=Path)
    args=parser.parse_args()
    if args.command=='create':create(args.destination,args.build)
    elif args.command=='verify':verify(args.directory)
    elif args.command=='extract':extract(args.source,args.destination)
    elif args.command=='install':install(args.source,args.destination)
    else:rollback(args.destination)
    print(args.command+' completed; no board was flashed')
if __name__=='__main__':main()
