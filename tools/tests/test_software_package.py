import importlib.util
import json
from pathlib import Path
import tempfile
import tarfile
import io
import unittest
spec=importlib.util.spec_from_file_location('package',Path(__file__).resolve().parents[2]/'scripts/software-package.py')
package=importlib.util.module_from_spec(spec);spec.loader.exec_module(package)

class PackageTests(unittest.TestCase):
    def bundle(self,path,value):
        path.mkdir();(path/'app.bin').write_bytes(value)
        (path/'manifest.json').write_text(json.dumps(dict(version=1,flight_qualified=False,profiles=[],files={'app.bin':package.sha(path/'app.bin')})))
    def test_replacement_and_verified_rollback(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);self.bundle(root/'a',b'old');self.bundle(root/'b',b'new')
            package.install(root/'a',root/'installed');package.install(root/'b',root/'installed')
            self.assertEqual((root/'installed/app.bin').read_bytes(),b'new')
            package.rollback(root/'installed')
            self.assertEqual((root/'installed/app.bin').read_bytes(),b'old')
            self.assertEqual((root/'installed.previous/app.bin').read_bytes(),b'new')
    def test_corruption_traversal_and_symlink_cannot_replace_working_install(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);self.bundle(root/'a',b'working');package.install(root/'a',root/'installed')
            (root/'a/app.bin').write_bytes(b'corrupt')
            with self.assertRaises(ValueError):package.install(root/'a',root/'installed')
            self.assertEqual((root/'installed/app.bin').read_bytes(),b'working')
            for name in ('../outside','/etc/passwd'):
                with self.assertRaises(ValueError):package.member(root,name)
            (root/'installed/hidden-link').symlink_to(root/'missing')
            with self.assertRaises(ValueError):package.verify(root/'installed')
            (root/'installed/hidden-link').unlink()
            (root/'link').symlink_to(root/'installed/app.bin')
            with self.assertRaises(ValueError):package.member(root,'link')

    def test_verified_archive_cannot_extract_outside_destination(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'bundle';source.mkdir()
            with tarfile.open(source/'source.tar.gz','w:gz') as archive:
                entry=tarfile.TarInfo('../escape');entry.size=1
                archive.addfile(entry,io.BytesIO(b'x'))
            (source/'source-files.json').write_text(json.dumps({'../escape':'unused'}))
            files={name:package.sha(source/name) for name in ('source.tar.gz','source-files.json')}
            (source/'manifest.json').write_text(json.dumps(dict(version=1,flight_qualified=False,profiles=[],files=files)))
            with self.assertRaises(ValueError):package.extract(source,root/'destination')
            self.assertFalse((root/'escape').exists());self.assertFalse((root/'destination').exists())
