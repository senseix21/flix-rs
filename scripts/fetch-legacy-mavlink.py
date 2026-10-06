#!/usr/bin/env python3
"""Fetch the exact Arduino MAVLink version pinned by the legacy Makefile."""
import gzip,hashlib,io,json
from pathlib import Path
import urllib.request
import zipfile
ROOT=Path(__file__).resolve().parents[1]
cache=ROOT/'.local/legacy-mavlink';cache.mkdir(parents=True,exist_ok=True)
record=cache/'provenance.json'
if record.exists() and (cache/'MAVLink/MAVLink.h').exists():
    print(record.read_text());raise SystemExit(0)
index=json.loads(gzip.decompress(urllib.request.urlopen('https://downloads.arduino.cc/libraries/library_index.json.gz',timeout=30).read()))
entry=next(x for x in index['libraries'] if x['name']=='MAVLink' and x['version']=='2.0.33')
archive=urllib.request.urlopen(entry['url'],timeout=30).read()
digest=hashlib.sha256(archive).hexdigest()
assert entry['checksum']=='SHA-256:'+digest
with zipfile.ZipFile(io.BytesIO(archive)) as source:
    for item in source.infolist():
        parts=Path(item.filename).parts
        assert '..' not in parts and not Path(item.filename).is_absolute()
        relative=Path(*parts[1:])
        dest=cache/'MAVLink'/relative
        if item.is_dir():dest.mkdir(parents=True,exist_ok=True)
        else:dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(source.read(item))
assert (cache/'MAVLink/MAVLink.h').exists()
record.write_text(json.dumps(dict(name=entry['name'],version=entry['version'],url=entry['url'],sha256=digest),indent=2)+'\n')
print(record.read_text())
