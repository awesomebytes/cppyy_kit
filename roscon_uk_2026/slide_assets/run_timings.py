"""Run from the repository's activated Pixi environment; keep raw terminal output."""
import datetime
import json
import os
from pathlib import Path
import platform
import statistics
import subprocess
import sys
import tempfile

ASSETS = Path(__file__).resolve().parent
REPO = ASSETS.parents[1]
cache = tempfile.mkdtemp(prefix='slide-timings-', dir=REPO / 'build')
env = {key: value for key, value in os.environ.items() if 'PCH' not in key}
env['CPPYY_KIT_CACHE_DIR'] = cache
report = {'recorded_at': datetime.datetime.now().astimezone().isoformat(),
          'python': sys.version, 'platform': platform.platform(),
          'cache': cache, 'startup': {}, 'throughput': {},
          'method': 'Sequential fresh processes; kit startup hook disabled for Python/cppyy baselines. '
                    'Vanilla cppyy uses its own standard PCH; kit uses its auto-PCH. Existing OS caches. '
                    'Cold means fresh kit function cache only. Seven warmed calls per throughput process.'}
transcript = []


def invoke(script, tiny=False):
    args = ['/usr/bin/time', '-f', 'process_elapsed_s=%e', sys.executable,
            str(ASSETS / script)] + (['--tiny'] if tiny else [])
    child_env = dict(env)
    if script != 'kit_sum.py':
        child_env['CPPYY_KIT_NO_AUTOPCH'] = '1'
    completed = subprocess.run(args, env=child_env, text=True, capture_output=True, check=True)
    entry = '$ ' + ' '.join(args) + '\n' + completed.stdout + completed.stderr
    print(entry, flush=True)
    transcript.append(entry)
    elapsed = float(completed.stderr.rsplit('process_elapsed_s=', 1)[1].strip())
    return completed.stdout.strip(), elapsed


for label, script in [('python', 'python_sum.py'), ('cppyy', 'cppyy_sum.py'),
                      ('kit', 'kit_sum.py')]:
    samples = []
    for index in range(4):
        output, elapsed = invoke(script, tiny=True)
        if output != '14.0':
            raise ValueError(output)
        samples.append(elapsed)
    report['startup'][label] = {'first_s': samples[0], 'repeat_s': samples[1:],
                                'repeat_median_s': statistics.median(samples[1:])}
for label, script in [('python', 'python_sum.py'), ('cppyy', 'cppyy_sum.py'),
                      ('kit', 'kit_sum.py')]:
    output, _ = invoke(script)
    report['throughput'][label] = json.loads(output)
(ASSETS / 'timings.json').write_text(json.dumps(report, indent=2) + '\n')
(ASSETS / 'terminal.txt').write_text('\n'.join(transcript))
