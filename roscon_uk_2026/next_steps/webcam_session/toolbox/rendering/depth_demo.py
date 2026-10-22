"""Compare unified viewers on the same webcam/DA3 pipeline."""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.session import Workbench
from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.depth_pipeline import DepthPipeline
from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.polyscope_view import PolyscopeViewer
from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.viser_view import ViserViewer


def measure(workbench, viewer, seconds):
    before, view_before = workbench.status(), viewer.status()
    start = time.monotonic()
    samples = []
    last = None
    while time.monotonic()-start < seconds:
        packet=workbench.latest(processed=True)
        if packet and packet['sequence'] != last:
            samples.append([packet['inference_ms'],packet['geometry_and_color_ms'],
                            (time.monotonic()-packet['stamp'])*1000])
            last=packet['sequence']
        time.sleep(0.005)
    elapsed=time.monotonic()-start
    after, view_after=workbench.status(),viewer.status()
    report=dict(seconds=elapsed,before=before,after=after,viewer_before=view_before,
                viewer_after=view_after,capture_hz=(after['captured']-before['captured'])/elapsed,
                processing_hz=(after['processed']-before['processed'])/elapsed)
    if samples:
        report['sample_count']=len(samples)
        report['median_ms']=dict(zip(['inference','geometry_and_color','observed_result_age'],np.median(samples,axis=0).tolist()))
        report['p95_ms']=dict(zip(['inference','geometry_and_color','observed_result_age'],np.percentile(samples,95,axis=0).tolist()))
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--viewer',choices=['viser','polyscope','compare'],default='viser')
    parser.add_argument('--seconds',type=float,default=0,help='0 runs until Ctrl-C')
    parser.add_argument('--camera',type=int,default=0)
    parser.add_argument('--stride',type=int,default=2)
    parser.add_argument('--calibration', type=Path, help='Webcam calibration JSON')
    parser.add_argument('--output',type=Path,default=Path('build/webcam_session/depth_comparison'))
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    pipeline=DepthPipeline(calibration=args.calibration)
    # Populate native cache and model kernels before opening a viewer.
    pipeline(np.zeros((360,640,3),np.uint8),{},dict(stride=args.stride))
    workbench=Workbench(camera=args.camera)
    workbench.configure(stride=args.stride)
    workbench.set_processor(pipeline)
    reports={}
    viewer=None
    try:
        for mode in (['viser','polyscope'] if args.viewer=='compare' else [args.viewer]):
            viewer=ViserViewer(workbench) if mode=='viser' else PolyscopeViewer(workbench)
            print(f'VIEWER_READY {mode}',flush=True)
            if mode=='viser':
                deadline=time.monotonic()+90
                while not viewer.server.get_clients() and time.monotonic()<deadline:
                    time.sleep(.2)
                print(f'CLIENTS {len(viewer.server.get_clients())}',flush=True)
            time.sleep(3)
            if args.seconds:
                reports[mode]=measure(workbench,viewer,args.seconds)
                if mode=='polyscope':
                    viewer.screenshot_path=str(args.output/'polyscope.png')
                    time.sleep(.3)
                print(json.dumps({mode:reports[mode]},indent=2),flush=True)
                (args.output/'results.json').write_text(json.dumps(reports,indent=2))
            else:
                while viewer.status().get("running", True):
                    time.sleep(1)
            viewer.close()
            viewer=None
    except KeyboardInterrupt:
        pass
    finally:
        if viewer:
            viewer.close()
        workbench.close()


if __name__=='__main__':
    main()
