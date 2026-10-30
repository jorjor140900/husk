"""Cloud transport must retain timing fields, signed bytes and row padding."""
import copy
import unittest
from types import SimpleNamespace as NS
from husky_link_core.cloud_wire import encode_cloud, decode_cloud, MAX_CLOUD_BYTES
from husky_link_core.protocol import dumps, loads
from test_gateway_media import Header
from test_vlp16 import message
from husky_drivable_area.vlp16 import read_timed_cloud


class CloudWireTests(unittest.TestCase):
    def source(self, endian='<', padded=False):
        source=message(endian,padded)
        source.header=Header(100_000_000_000)
        source.header.frame_id='velodyne'
        source.is_dense=True
        source.fields=[dict(name=f.name,offset=f.offset,datatype=f.datatype,count=f.count) for f in source.fields]
        return source

    def test_lossless_bytes_fields_signed_time_and_padding(self):
        for endian in ('<','>'):
            for padded in (False,True):
                source=self.source(endian,padded)
                value=loads(dumps(encode_cloud(source)))
                meta,raw=decode_cloud(value)
                self.assertEqual(raw,source.data)
                self.assertEqual(meta['fields'],source.fields)
                meta['fields']=[NS(**f) for f in meta['fields']]
                meta['header']=source.header
                decoded=read_timed_cloud(NS(**meta,data=raw))
                self.assertAlmostEqual(decoded.offsets_sec[0],-.1)
                self.assertEqual(decoded.rings.tolist(),[0,15])

    def test_malformed_layout_base64_size_and_truncation_rejected(self):
        original=encode_cloud(self.source())
        for kind in ('flags','length','base64','row','step','duplicate','huge','key'):
            value=copy.deepcopy(original)
            if kind=='flags':value['is_dense']=1
            if kind=='length':value['data_b64']=''
            if kind=='base64':value['data_b64']='!bad!'
            if kind=='row':value['row_step']=1
            if kind=='step':value['point_step']=0
            if kind=='duplicate':value['fields'].append(value['fields'][0])
            if kind=='huge':value['height']=MAX_CLOUD_BYTES
            if kind=='key':value['extra']=1
            with self.subTest(kind=kind),self.assertRaises(ValueError):decode_cloud(value)
