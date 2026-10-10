"""Run a native filter over typed MCAP and write portable error reports."""
import argparse
import base64
import hashlib
import html
import importlib.metadata
import json
import os
import platform
from pathlib import Path
import struct
import shlex
import subprocess
import sys
import time
import zlib

import numpy as np
from .report_core import MAX_SAMPLES, check_frames, match_images, rank_windows, trace_svg

CLOCK_FIELDS = {'header': 'header_time_ns', 'publish': 'publish_time_ns', 'log': 'log_time_ns'}
MAX_IMAGE_BYTES = 16_000_000
MAX_DATASET_BYTES = 128_000_000
MAX_HTML_BYTES = 32_000_000


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def hash_json(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',',':'), allow_nan=False).encode()).hexdigest()


def clock_times(batch, clock, *, strict):
    from .report_core import integer_times
    times = integer_times(getattr(batch, CLOCK_FIELDS[clock]), strict=strict)
    if any(t < -(1 << 63) or t >= (1 << 63) for t in times):
        raise ValueError('selected clock cannot be represented by native int64 nanoseconds')
    return times


def png_rgb(width, height, step, encoding, payload):
    """Convert only selected raw rgb8/mono8 frames to lossless portable PNG."""
    width, height, step = int(width), int(height), int(step)
    channels = {'rgb8': 3, 'mono8': 1}.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported report image encoding: {encoding}')
    if width <= 0 or height <= 0 or width*height*channels > MAX_IMAGE_BYTES or step < width*channels:
        raise ValueError('invalid or oversized image dimensions/step')
    payload = bytes(payload)
    if len(payload) != step*height or len(payload) > MAX_IMAGE_BYTES:
        raise ValueError('image data length/step mismatch or byte limit exceeded')
    raw = b''.join(b'\0'+payload[row*step:row*step+width*channels] for row in range(height))
    def chunk(tag, data):
        return struct.pack('>I', len(data))+tag+data+struct.pack('>I',zlib.crc32(tag+data)&0xffffffff)
    color_type = 2 if channels == 3 else 0
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,height,8,color_type,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b'')


def render(report, times, errors, observation_errors, images):
    esc = lambda value: html.escape(str(value), quote=True)
    parts = ['<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>Native estimator dataset report</title>',
             '<style>body{font:16px system-ui;max-width:1050px;margin:2rem auto;padding:0 1rem}table{border-collapse:collapse}td,th{padding:.4rem;border:1px solid #aaa;text-align:left}pre{white-space:pre-wrap;overflow-wrap:anywhere}img{max-width:100%;image-rendering:pixelated}svg{width:100%}.missing{color:#b21}</style><h1>Native estimator dataset report</h1>',
             f'<p>Episode: {esc(report["episode_id"])}. Ranking: {esc(report["metric"]["label"])}. Frame: {esc(report["frame_id"])}. Metres.</p>',
             '<p>Clock and nanosecond timestamps remain explicit in the tables. The plot uses relative seconds after integer subtraction.</p>',
             trace_svg(times, [('filtered diagnostic residual' if report['metric']['reference_topic'] is None else 'filtered error', errors), ('observation residual' if report['metric']['reference_topic'] is None else 'observation error', observation_errors)]),
             f'<p>Image alignment: {esc(report["alignment"])}.</p>',
             '<table><thead><tr><th>Rank</th><th>[start, end) ns</th><th>Mean error m</th><th>Samples</th><th>Representative ns</th><th>Image ns / delta ns</th></tr></thead><tbody>']
    for rank, window in enumerate(report['windows'], 1):
        image = window['image']
        parts.append(f'<tr><td>{rank}</td><td>{window["start_ns"]}, {window["end_ns"]}</td><td>{window["score"]:.9g}</td><td>{window["samples"]}</td><td>{window["representative_ns"]}</td><td>{esc(image["image_ns"])} / {esc(image["delta_ns"])} ({esc(image["status"])})</td></tr>')
    parts.append('</tbody></table>')
    for rank, window in enumerate(report['windows'], 1):
        image = window['image']
        parts.append(f'<h2>Window {rank}</h2>')
        if image['status'] == 'matched':
            parts.append(f'<img alt="Representative camera frame for window {rank}" src="data:image/png;base64,{base64.b64encode(images[image["image_index"]]).decode()}"/>')
            parts.append(f'<p>Camera frame: {esc(image.get("frame_id", "unknown"))}. Image frame is not the pose coordinate frame; no extrinsic transform is applied.</p>')
        else:
            parts.append('<p class="missing">Missing image: no frame within the declared direction and tolerance.</p>')
    parts.append('<h2>Provenance and regeneration</h2><pre>'+esc(json.dumps(report['provenance'],indent=2,ensure_ascii=True))+'</pre>')
    parts.append('<h2>Stage timings, seconds</h2><pre>'+esc(json.dumps(report['timings_s'],indent=2))+'</pre></html>')
    result = ''.join(parts)
    if len(result.encode()) > MAX_HTML_BYTES:
        raise ValueError('portable HTML exceeds byte limit')
    return result


def build_report(args):
    from roscon_uk_2026.next_steps.reverse_core import FilterConfig, PoseFilter, build_native, load_config
    from roscon_uk_2026.next_steps.typed_mcap import extract
    dataset = Path(args.dataset).resolve()
    if dataset.stat().st_size > MAX_DATASET_BYTES:
        raise ValueError('dataset exceeds 128 MB demo limit')
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    timings = {}
    start = time.perf_counter()
    metadata_path = Path(args.metadata).resolve() if args.metadata else dataset.with_suffix('.json')
    if metadata_path.exists() and metadata_path.stat().st_size > 1_000_000:
        raise ValueError('metadata exceeds 1 MB demo limit')
    metadata = json.loads(metadata_path.read_text()) if metadata_path.exists() else {}
    if not isinstance(metadata,dict):
        raise ValueError('metadata must be a JSON object')
    dataset_hash = sha256(dataset)
    diagnostic = getattr(args, 'diagnostic', False)
    if metadata.get('mcap_sha256') and metadata['mcap_sha256'] != dataset_hash:
        raise ValueError('metadata dataset checksum mismatch')
    if not diagnostic and (metadata.get('truth_kind') != 'analytic synthetic trajectory' or metadata.get('mcap_sha256') != dataset_hash):
        raise ValueError('error ranking requires checksum-bound analytic synthetic reference metadata; use --diagnostic for an unreferenced residual heuristic')
    timings['metadata_io_and_dataset_hash'] = time.perf_counter()-start
    start = time.perf_counter()
    native_paths = build_native()
    timings['native_build_or_cache'] = time.perf_counter()-start
    start = time.perf_counter()
    poses = extract.extract_poses(dataset, topic=args.pose_topic)
    truth = poses if diagnostic else extract.extract_poses(dataset, topic=args.truth_topic)
    timings['typed_pose_and_truth_extraction'] = time.perf_counter()-start
    start = time.perf_counter()
    times = clock_times(poses,args.clock,strict=True)
    truth_times = clock_times(truth,args.clock,strict=True)
    if times != truth_times:
        raise ValueError('truth timestamps must exactly match pose timestamps in the selected clock')
    if len(times) < 2 or len(times) > MAX_SAMPLES:
        raise ValueError('report requires 2 to 1000000 pose samples')
    config = load_config(args.config) if args.config else FilterConfig(frame_id=poses.frame_id)
    check_frames(poses.frame_id,truth.frame_id,config.frame_id)
    position = np.ascontiguousarray(poses.position_m, dtype=np.float64)
    reference = np.ascontiguousarray(truth.position_m, dtype=np.float64)
    timestamp_array = np.asarray(times,dtype=np.int64)
    if reference.shape != position.shape or position.shape != (len(times),3) or not np.isfinite(reference).all():
        raise ValueError('pose and truth arrays must be finite Nx3 Cartesian metres')
    if not diagnostic:
        header_times = clock_times(truth,'header',strict=True)
        relative_s = np.asarray([t-header_times[0] for t in header_times],dtype=np.int64)*1e-9
        analytic_truth = np.column_stack((0.2*relative_s,0.1*np.sin(relative_s),0.03*np.cos(2*relative_s)))
        if not np.allclose(reference,analytic_truth,rtol=0,atol=1e-14):
            raise ValueError('reference does not match the declared analytic synthetic trajectory')
    timings['validation_and_input_conversion'] = time.perf_counter()-start
    start = time.perf_counter()
    with PoseFilter(config) as estimator:
        timings['native_load_and_construction'] = time.perf_counter()-start
        start = time.perf_counter()
        filtered = estimator.process(timestamp_array,position)
        timings['native_batch_and_output_conversion'] = time.perf_counter()-start
        start = time.perf_counter()
        snapshot = estimator.snapshot()
    timings['native_snapshot_and_close'] = time.perf_counter()-start
    start = time.perf_counter()
    errors = np.linalg.norm(filtered-reference,axis=1).tolist()
    observation_errors = np.linalg.norm(position-reference,axis=1).tolist()
    windows = rank_windows(times,errors,args.duration_ns)
    timings['error_computation_and_window_ranking'] = time.perf_counter()-start
    start = time.perf_counter()
    # Metadata-only indexing is used when the extractor supplies it.
    header_only = hasattr(extract,'extract_image_headers')
    image_headers = (extract.extract_image_headers if header_only else extract.extract_images)(dataset,topic=args.image_topic)
    image_times = clock_times(image_headers,args.clock,strict=False)
    matches = match_images([w.representative_ns for w in windows],image_times,
                           query_clock=args.clock,image_clock=args.clock,
                           direction=args.direction,tolerance_ns=args.tolerance_ns)
    timings['image_metadata_extraction_and_integer_join'] = time.perf_counter()-start
    start = time.perf_counter()
    images = {}
    image_payload_timings = []
    for match in matches:
        index = match['image_index']
        if index is None or index in images:
            continue
        if header_only:
            # Extractor selection uses log_time regardless of the join clock.
            log_time = int(image_headers.log_time_ns[index])
            batch = extract.extract_images(dataset,topic=args.image_topic,start_ns=log_time,end_ns=log_time+1)
            image_payload_timings.append(batch.timings_ms)
            candidates = [i for i,t in enumerate(clock_times(batch,args.clock,strict=False)) if t == match['image_ns']]
            if not candidates:
                raise ValueError('selected image disappeared between metadata and payload extraction')
            # Preserve duplicate identity by its source position at this log time.
            same_log_before = sum(int(x)==log_time for x in image_headers.log_time_ns[:index])
            local_index = same_log_before
            if local_index not in candidates:
                raise ValueError('selected image duplicate identity mismatch')
        else:
            batch,local_index = image_headers,index
        offsets = batch.offsets
        payload = batch.data[int(offsets[local_index]):int(offsets[local_index+1])]
        encoding = batch.encoding if isinstance(batch.encoding,str) else batch.encoding[local_index]
        images[index] = png_rgb(batch.width[local_index],batch.height[local_index],batch.step[local_index],encoding,payload)
    for match in matches:
        frame = getattr(image_headers,'frame_id','unknown')
        match['frame_id'] = frame if isinstance(frame,str) else (frame[match['image_index']] if match['image_index'] is not None else None)
    timings['selected_image_payload_extraction_and_png_encoding'] = time.perf_counter()-start
    compiler = shlex.split(os.environ.get('CXX','c++'))
    versions = {'python':platform.python_version(),'platform':platform.platform(),
                'compiler':subprocess.check_output([*compiler,'--version'],text=True).splitlines()[0],
                'mcap_cpp_vendor':'0.26.11 (locked ros-jazzy-mcap-vendor)',
                'cppyy_kit':'repository source; source hash recorded'}
    for package in ('numpy','cppyy','pydantic','mcap','mcap-ros2-support'):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = 'not installed'
    config_json = config.model_dump(mode='json')
    source_paths = [Path(__file__),Path(__file__).with_name('report_core.py'),Path(__file__).parents[3]/'cppyy_kit/__init__.py']
    source_paths += sorted(Path(extract.__file__).parent.glob('*.py'))+sorted(Path(extract.__file__).parent.glob('native.*'))
    source_paths += [Path(__file__).with_name('pixi.lock')]
    core_dir = Path(__file__).parents[1]/'reverse_core'
    source_paths += sorted(core_dir.glob('*.py'))+sorted((core_dir/'native').glob('*'))
    source_hashes = {str(p.relative_to(Path(__file__).parents[3])):sha256(p) for p in source_paths if p.is_file()}
    command = [sys.executable,'-m','roscon_uk_2026.next_steps.reports.report',str(dataset),'--output',str(output),'--clock',args.clock,'--duration-ns',str(args.duration_ns),'--tolerance-ns',str(args.tolerance_ns),'--direction',args.direction,'--pose-topic',args.pose_topic,'--truth-topic',args.truth_topic,'--image-topic',args.image_topic]
    if diagnostic:
        command += ['--diagnostic']
    if args.config:
        command += ['--config',str(Path(args.config).resolve())]
    if metadata_path.exists():
        command += ['--metadata',str(metadata_path)]
    report = {'report_version':1,'episode_id':metadata.get('episode_id',dataset.stem),'frame_id':poses.frame_id,
              'metric':{'label':'mean observation residual diagnostic heuristic' if diagnostic else 'mean synthetic-reference position error','units':'metres','reference_topic':None if diagnostic else args.truth_topic,'reference_verification':'unreferenced diagnostic; not ground-truth error' if diagnostic else 'checksum-bound sidecar; analytic trajectory verified to 1e-14 m; timestamp equality and frame equality','selection':'greedy descending mean, score tie rounded to 1e-12 m then earliest start; half-open fixed-duration sample-start candidates; disjoint; peak-error representative','duration_ns':args.duration_ns},
              'alignment':{'clock':args.clock,'declared_clock':metadata.get(args.clock+'_clock','unspecified'),'direction':args.direction,'tolerance_ns':args.tolerance_ns,'nearest_tie':'earlier time, then first source duplicate','backward_duplicate':'last source duplicate','image_metadata_only':header_only},
              'windows':[dict(w.as_dict(),image=match) for w,match in zip(windows,matches)],
              'extractor_timings_ms':{'setup':dict(extract.SETUP_TIMINGS),'poses':poses.timings_ms,'truth':None if diagnostic else truth.timings_ms,'image_headers':image_headers.timings_ms,'selected_payloads':image_payload_timings},
              'sample_count':len(times),'image_count':len(image_times),'selected_unique_images':len(images),'native_snapshot':snapshot,
              'provenance':{'dataset_path':str(dataset),'dataset_sha256':dataset_hash,'metadata':metadata,'metadata_sha256':sha256(metadata_path) if metadata_path.exists() else None,'config':config_json,'config_sha256':hash_json(config_json),'versions':versions,'source_sha256':source_hashes,'native_library_sha256':sha256(native_paths['library']),'regenerate_argv':command},
              'timings_s':timings}
    start = time.perf_counter()
    rendered = render(report,times,errors,observation_errors,images)
    timings['html_render'] = time.perf_counter()-start
    # Include final measured rendering time in the portable document.
    rendered = render(report,times,errors,observation_errors,images)
    start = time.perf_counter()
    (output/'report.html').write_text(rendered,encoding='utf-8')
    traces = {'timestamps_ns':times,('diagnostic_residual_m' if diagnostic else 'error_m'):errors,('observation_residual_m' if diagnostic else 'observation_error_m'):observation_errors}
    (output/'traces.json').write_text(json.dumps(traces,allow_nan=False))
    timings['html_and_trace_write'] = time.perf_counter()-start
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset')
    parser.add_argument('--output',default='roscon_uk_2026/next_steps/reports/build/report')
    parser.add_argument('--metadata')
    parser.add_argument('--diagnostic',action='store_true',help='rank unreferenced observation residuals; these are not ground-truth errors')
    parser.add_argument('--config')
    parser.add_argument('--clock',choices=CLOCK_FIELDS,default='header')
    parser.add_argument('--direction',choices=('nearest','backward','forward'),default='nearest')
    parser.add_argument('--duration-ns',type=int,default=200_000_000)
    parser.add_argument('--tolerance-ns',type=int,default=60_000_000)
    parser.add_argument('--pose-topic',default='/pose')
    parser.add_argument('--truth-topic',default='/truth')
    parser.add_argument('--image-topic',default='/camera/image')
    args = parser.parse_args()
    report = build_report(args)
    print(json.dumps({'windows':len(report['windows']),'selected_images':report['selected_unique_images'],'unmatched':sum(w['image']['status']=='unmatched' for w in report['windows']),'timings_s':report['timings_s']},indent=2))


if __name__ == '__main__':
    main()
