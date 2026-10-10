"""Run uncertain header parsing and autodiff instantiation in a subprocess."""
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
CODE = r'''
#define GLOG_USE_GLOG_EXPORT
#include <ceres/ceres.h>
#include <manif/SO3.h>
#include <manif/ceres/ceres.h>
struct ProbeResidual {
  template <typename T> bool operator()(const T* x, T* r) const {
    Eigen::Matrix<T,3,1> w(x[3],x[4],x[5]), p(T(1),T(2),T(3));
    auto y = manif::SO3Tangent<T>(w).exp().act(p);
    for(int i=0;i<3;++i) r[i]=y[i]+x[i]-T(i+1);
    return true;
  }
};
double probe_solve() {
  double x[6] = {.1,.1,.1,0,0,0};
  ceres::Problem p;
  p.AddResidualBlock(new ceres::AutoDiffCostFunction<ProbeResidual,3,6>(new ProbeResidual),nullptr,x);
  ceres::Solver::Options o; o.linear_solver_type=ceres::DENSE_QR;
  ceres::Solver::Summary s; ceres::Solve(o,&p,&s);
  return s.final_cost;
}
int probe_standard(){ return __cplusplus; }
int probe_alignment(){ return EIGEN_MAX_ALIGN_BYTES; }
int probe_abi(){ return _GLIBCXX_USE_CXX11_ABI; }
'''

def child():
    code=CODE
    if "--missing-exports" in sys.argv: code=code.replace("#define GLOG_USE_GLOG_EXPORT", "")
    if "--missing-manif-helper" in sys.argv: code=code.replace("#include <manif/ceres/ceres.h>", "")
    if "--compile" in sys.argv:
        from cppyy_kit._compile import compile_shared
        (ROOT/"build").mkdir(exist_ok=True)
        source=ROOT/"build/probe.cpp"
        source.write_text(code)
        library=ROOT/"build/libprobe.so"
        compile_shared(str(source),str(library),
                       include_paths=[str(Path(sys.prefix)/"include"),str(Path(sys.prefix)/"include/eigen3")],
                       library_paths=[str(Path(sys.prefix)/"lib")],libraries=["ceres","glog","gflags"])
    import cppyy
    if "--compile" in sys.argv:
        cppyy.load_library(str(library))
        cppyy.cppdef("double probe_solve(); int probe_standard(); int probe_alignment(); int probe_abi();")
    else:
        cppyy.add_include_path(str(Path(sys.prefix) / "include"))
        cppyy.add_include_path(str(Path(sys.prefix) / "include/eigen3"))
        cppyy.load_library(str(Path(sys.prefix) / "lib/libceres.so"))
        cppyy.cppdef(code)
    print(json.dumps({"cost": cppyy.gbl.probe_solve(),
                      "cplusplus": cppyy.gbl.probe_standard(),
                      "eigen_max_align_bytes": cppyy.gbl.probe_alignment(),
                      "glibcxx_cxx11_abi": cppyy.gbl.probe_abi()}))

def run():
    start = time.perf_counter()
    try:
        args = [arg for arg in ("--missing-exports","--missing-manif-helper","--compile") if arg in sys.argv]
        p = subprocess.run([sys.executable, str(Path(__file__).resolve()), "--child", *args],
                           capture_output=True, text=True, timeout=90)
        result = dict(returncode=p.returncode, stdout=p.stdout, stderr=p.stderr,
                      seconds=time.perf_counter()-start)
    except subprocess.TimeoutExpired as e:
        result = dict(returncode=None, stdout=str(e.stdout), stderr=str(e.stderr),
                      seconds=time.perf_counter()-start, timeout_seconds=90)
    (ROOT / "build").mkdir(exist_ok=True)
    name = "probe_missing_exports.json" if "--missing-exports" in sys.argv else "probe.json"
    if "--missing-manif-helper" in sys.argv: name="probe_missing_manif_helper.json"
    if "--compile" in sys.argv: name=name.replace(".json","_compile.json")
    (ROOT / "build" / name).write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k != "stderr"}, indent=2))
    print("Full stderr saved in build/"+name)
    return result

if __name__ == "__main__":
    child() if "--child" in sys.argv else run()
