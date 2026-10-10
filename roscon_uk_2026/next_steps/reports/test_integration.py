"""Native MCAP integration with independent analytic reference and join oracle."""
import argparse
import base64
import re
import json
from pathlib import Path
import struct
import tempfile
import unittest
import zlib

import numpy as np
from roscon_uk_2026.next_steps.typed_mcap.extract import generate_fixture, extract_poses, extract_image_headers
from .report import build_report, png_rgb
from .report_core import match_images


class NativeReportChecks(unittest.TestCase):
    def test_generated_report_regenerates_and_matches_reference(self):
        (Path(__file__).parent/'build').mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent/'build') as temp:
            root = Path(temp)
            dataset = root/'episode.mcap'
            generate_fixture(dataset)
            initial_sidecar = dataset.with_suffix('.json')
            initial_metadata = json.loads(initial_sidecar.read_text())
            initial_metadata['episode_id'] = '<script>alert(1)</script> & \"episode\"'
            initial_sidecar.write_text(json.dumps(initial_metadata))
            args = argparse.Namespace(dataset=str(dataset),output=str(root/'output'),metadata=None,
                                      config=None,clock='header',duration_ns=200_000_000,
                                      tolerance_ns=60_000_000,direction='nearest',pose_topic='/pose',
                                      truth_topic='/truth',image_topic='/camera/image')
            report = build_report(args)
            traces = json.loads((root/'output/traces.json').read_text())
            base = 1_700_000_000_000_000_000
            times = traces['timestamps_ns']
            self.assertEqual(times, [base+i*10_000_000 for i in range(600)])
            truth = extract_poses(dataset, topic='/truth').position_m
            t = np.arange(600)*0.01
            expected_truth = np.column_stack((0.2*t,0.1*np.sin(t),0.03*np.cos(2*t)))
            np.testing.assert_allclose(truth,expected_truth,rtol=0,atol=1e-15)
            observed = extract_poses(dataset).position_m
            reference_filter = observed.copy()
            alpha = -np.expm1(-0.01/0.08)
            for i in range(1,600):
                reference_filter[i] = reference_filter[i-1]+alpha*(observed[i]-reference_filter[i-1])
            np.testing.assert_allclose(traces['error_m'], np.linalg.norm(reference_filter-expected_truth,axis=1),rtol=0,atol=1e-14)
            headers = extract_image_headers(dataset)
            gap_match = match_images([base+2_500_000_000],headers.header_time_ns,query_clock='header',image_clock='header')
            self.assertEqual(gap_match[0]['status'],'unmatched')
            # Independent fixed-rate window scoring and greedy interval oracle.
            errors = traces['error_m']
            candidates = [(sum(errors[i:i+20])/20, i) for i in range(580)]
            candidates.sort(key=lambda item:(-round(item[0],12),item[1]))
            chosen = []
            for score,index in candidates:
                if all(index+20 <= other or other+20 <= index for other in chosen):
                    chosen.append(index)
                    if len(chosen)==5:
                        break
            self.assertEqual([w['start_ns'] for w in report['windows']],[times[i] for i in chosen])
            camera = [(base+i*10_000_000,i) for i in range(0,600,10) if not 200 <= i < 300]
            for window in report['windows']:
                start_index = (window['start_ns']-base)//10_000_000
                peak = max(range(start_index,start_index+20),key=lambda i:(errors[i],-i))
                self.assertEqual(window['representative_ns'],times[peak])
                nearest = min(camera,key=lambda row:(abs(row[0]-times[peak]),row[0]))
                expected = nearest[0] if abs(nearest[0]-times[peak])<=60_000_000 else None
                self.assertEqual(window['image']['image_ns'],expected)
            document = (root/'output/report.html').read_text()
            self.assertNotIn('https://', document)
            self.assertNotIn('<script>',document)
            self.assertIn('&lt;script&gt;',document)
            self.assertIn('data:image/png;base64,', document)
            encoded_frames = re.findall(r'src="data:image/png;base64,([^"]+)"',document)
            matched_windows = [w for w in report['windows'] if w['image']['status']=='matched']
            self.assertEqual(len(encoded_frames),len(matched_windows))
            for encoded,window in zip(encoded_frames,matched_windows):
                frame = base64.b64decode(encoded)
                offset, compressed = 8, b''
                while offset < len(frame):
                    length = struct.unpack('>I',frame[offset:offset+4])[0]
                    if frame[offset+4:offset+8] == b'IDAT':
                        compressed += frame[offset+8:offset+8+length]
                    offset += length+12
                raw = zlib.decompress(compressed)
                camera_index = (window['image']['image_ns']-base)//10_000_000
                self.assertEqual(raw[:4],bytes([0,camera_index%256,0,0]))
            repeated = build_report(args)
            self.assertEqual(report['windows'],repeated['windows'])
            self.assertEqual(report['provenance']['dataset_sha256'],repeated['provenance']['dataset_sha256'])
            self.assertEqual(report['provenance']['config_sha256'],repeated['provenance']['config_sha256'])
            self.assertEqual(report['provenance']['source_sha256'],repeated['provenance']['source_sha256'])
            # Enforce missing-image visibility with a zero-tolerance report.
            args.tolerance_ns = 0
            missing = build_report(args)
            self.assertTrue(any(w['image']['status']=='unmatched' for w in missing['windows']))
            self.assertIn('Missing image:', (root/'output/report.html').read_text())
            args.truth_topic = '/pose'
            with self.assertRaisesRegex(ValueError,'analytic synthetic trajectory'):
                build_report(args)
            args.truth_topic = '/truth'
            # Metadata cannot silently bind a reference to another dataset.
            sidecar = dataset.with_suffix('.json')
            metadata = json.loads(sidecar.read_text())
            metadata['mcap_sha256'] = 'bad-checksum'
            sidecar.write_text(json.dumps(metadata))
            with self.assertRaisesRegex(ValueError,'checksum mismatch'):
                build_report(args)
            sidecar.unlink()
            with self.assertRaisesRegex(ValueError,'reference metadata'):
                build_report(args)
            args.diagnostic = True
            args.truth_topic = '/no_reference'
            diagnostic = build_report(args)
            self.assertIn('diagnostic heuristic',diagnostic['metric']['label'])
            self.assertIsNone(diagnostic['metric']['reference_topic'])
            self.assertIn('not ground-truth error',diagnostic['metric']['reference_verification'])

    def test_png_padding_and_length_guards(self):
        payload = bytes([1,2,3,99,4,5,6,99])
        png = png_rgb(1,2,4,'rgb8',payload)
        self.assertEqual(png[:8], b'\x89PNG\r\n\x1a\n')
        position = 8
        data = b''
        while position < len(png):
            size = struct.unpack('>I',png[position:position+4])[0]
            tag = png[position+4:position+8]
            chunk = png[position+8:position+8+size]
            crc = struct.unpack('>I',png[position+8+size:position+12+size])[0]
            self.assertEqual(zlib.crc32(tag+chunk)&0xffffffff,crc)
            if tag == b'IDAT':
                data += chunk
            position += 12+size
        self.assertEqual(zlib.decompress(data),bytes([0,1,2,3,0,4,5,6]))
        with self.assertRaises(ValueError):
            png_rgb(1,2,4,'rgb8',payload[:-1])
        with self.assertRaises(ValueError):
            png_rgb(1,2,4,'bgr8',payload)


if __name__ == '__main__':
    (Path(__file__).parent/'build').mkdir(exist_ok=True)
    unittest.main()
