"""Capture local first-use and warmed stage timings without performance claims."""
import argparse
import hashlib
import json
from pathlib import Path
import resource
import time


def main():
    here = Path(__file__).resolve().parent
    started = time.perf_counter()
    from .report import build_report
    from roscon_uk_2026.next_steps.typed_mcap.extract import generate_fixture
    import_s = time.perf_counter()-started
    build = here/'build'
    build.mkdir(exist_ok=True)
    dataset = build/'fixture.mcap'
    started = time.perf_counter()
    generate_fixture(dataset)
    fixture_s = time.perf_counter()-started
    args = argparse.Namespace(dataset=str(dataset),output=str(build/'report'),metadata=None,
                              config=None,clock='header',duration_ns=200_000_000,
                              tolerance_ns=60_000_000,direction='nearest',pose_topic='/pose',
                              truth_topic='/truth',image_topic='/camera/image',diagnostic=False)
    runs = []
    for _ in range(3):
        started = time.perf_counter()
        report = build_report(args)
        runs.append({'elapsed_s':time.perf_counter()-started,'stages_s':dict(report['timings_s']),
                     'extractor_ms':report['extractor_timings_ms']})
    checks = {}
    for name in ('test_report.py','test_integration.py'):
        checks[name] = hashlib.sha256((here/name).read_bytes()).hexdigest()
    evidence = {'measurement':'one local process; cached native builds; first report then two warmed reports',
                'import_s':import_s,'fixture_generation_s':fixture_s,'peak_rss_kib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                'runs':runs,'windows':report['windows'],'dataset_sha256':report['provenance']['dataset_sha256'],
                'config':report['provenance']['config'],'config_sha256':report['provenance']['config_sha256'],
                'versions':report['provenance']['versions'],'source_sha256':report['provenance']['source_sha256'],
                'native_library_sha256':report['provenance']['native_library_sha256'],'acceptance_checks_sha256':checks,
                'agent_evaluation':'not run; skeleton checks intentionally fail until logic is completed'}
    (here/'EVIDENCE.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps({'runs_elapsed_s':[r['elapsed_s'] for r in runs], 'peak_rss_kib':evidence['peak_rss_kib'],'evidence_path':str(here/'EVIDENCE.json')},indent=2))


if __name__ == '__main__':
    main()
