"""Run potentially unsafe cppyy declarations in disposable subprocesses."""
import json
import subprocess
import sys
from build import ROOT, SOURCE, build


def main():
    build_result = build()
    child = '''from native import session, set_target, vector
s = session(3)
set_target(s, [.5, -.25, .1])
a = s.step(vector([0., 0., 0.]))
assert len(a.position) == 3 and a.new_calculation
assert len(s.knots()) > 1
assert len(s.at_time(0.0).position) == 3
print('compiled adapter: PASS')
'''
    result = subprocess.run([sys.executable, '-c', child], cwd=ROOT, text=True, capture_output=True)
    report = {'build': build_result, 'adapter_returncode': result.returncode,
              'adapter_stdout': result.stdout, 'adapter_stderr': result.stderr}
    # This probe is optional. The compiled adapter remains the supported path.
    template = f'''import cppyy
cppyy.add_include_path({str(SOURCE / 'include')!r})
cppyy.include('ruckig/ruckig.hpp')
cppyy.cppdef('namespace probe {{ ruckig::Ruckig<3> engine(0.001); }}')
print('Ruckig template declaration: PASS')
'''
    result2 = subprocess.run([sys.executable, '-c', template], cwd=ROOT, text=True, capture_output=True)
    report.update(template_returncode=result2.returncode, template_stdout=result2.stdout,
                  template_stderr=result2.stderr)
    (ROOT / 'probe_results.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if result.returncode:
        raise SystemExit('compiled adapter failed')


if __name__ == '__main__':
    main()
