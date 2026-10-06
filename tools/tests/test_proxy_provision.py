import importlib.util
from pathlib import Path
import unittest
spec=importlib.util.spec_from_file_location('proxy_provision',Path(__file__).resolve().parents[2]/'scripts/proxy-provision.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class ProxyProvisionTests(unittest.TestCase):
    def test_profile_validation_rejects_ambiguous_peers_keys_and_channels(self):
        valid=dict(peer_mac='02:01:02:03:04:05',pmk='11'*16,lmk='22'*16,channel=6)
        self.assertEqual(module.validate(valid)[3],6)
        for field,value in [('peer_mac','01:01:02:03:04:05'),('peer_mac','00'*6),('pmk','00'*16),('lmk','22'*15),('channel',True),('channel',0),('channel',15)]:
            with self.subTest(field=field,value=value),self.assertRaises(ValueError):module.validate(dict(valid,**{field:value}))
        with self.assertRaises(ValueError):module.validate(dict(valid,discovery=True))
