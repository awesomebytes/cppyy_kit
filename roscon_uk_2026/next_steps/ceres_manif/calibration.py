"""Estimate T_target_source and compare a matching grouped Huber objective."""
import json
from pathlib import Path
import time
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from native import load, ROOT

TRUTH = np.array([0.30, -0.20, 0.50, 0.25, -0.35, 0.18])
TOLERANCE = 1e-12
HUBER_SCALE_M = 0.03

def observations(noise_m=0.0, outliers=False, seed=2026):
    """Twelve known varied poses observe twenty noncoplanar points each."""
    rng = np.random.default_rng(seed)
    landmarks = rng.uniform(-1,1,(20,3))
    point_sets = []
    for _ in range(12):
        motion = Rotation.from_rotvec(rng.uniform(-0.8,0.8,3))
        point_sets.append(motion.apply(landmarks)+rng.uniform(-0.5,0.5,3))
    source = np.ascontiguousarray(np.vstack(point_sets))
    target = Rotation.from_rotvec(TRUTH[3:]).apply(source)+TRUTH[:3]
    target += rng.normal(0,noise_m,target.shape)
    if outliers:
        indices = rng.choice(len(source), len(source)//10, replace=False)
        target[indices] += rng.normal(0,0.4,(len(indices),3))
    return source, np.ascontiguousarray(target)

def raw_residual(x, source, target):
    return Rotation.from_rotvec(x[3:]).apply(source)+x[:3]-target

def grouped_huber_residual(x, source, target, scale):
    residual = raw_residual(x,source,target)
    if scale == 0:
        return residual.ravel()
    squared_norm = np.sum(residual**2,axis=1)
    rho = np.where(squared_norm <= scale**2, squared_norm,
                   2*scale*np.sqrt(squared_norm)-scale**2)
    weight = np.sqrt(np.divide(rho,squared_norm,out=np.ones_like(rho),where=squared_norm>0))
    return (residual*weight[:,None]).ravel()

def objective(x, source, target, scale):
    r = grouped_huber_residual(x,source,target,scale)
    return float(0.5*r@r)

def validate(source,target,x):
    for name, value, shape in (("source",source,(-1,3)),("target",target,source.shape),("x",x,(6,))):
        if value.dtype!=np.float64 or not value.flags.c_contiguous:
            raise ValueError(name+" must be a C-contiguous float64 array")
        if not np.isfinite(value).all():
            raise ValueError(name+" must contain finite numbers")
        if (name=="source" and (value.ndim!=2 or value.shape[1]!=3)) or (name!="source" and value.shape!=shape):
            raise ValueError(name+" has an invalid shape")

def native_solve(api,source,target,scale=0.0,max_iterations=100,x=None):
    x = np.zeros(6) if x is None else np.array(x,dtype=np.float64,copy=True)
    validate(source,target,x)
    if len(source)<3:
        raise ValueError("at least three points are required")
    start = time.perf_counter()
    report = api.solve(source.ravel(),target.ravel(),len(source),x,
                       scale,max_iterations,TOLERANCE)
    elapsed = time.perf_counter()-start
    result = {name: getattr(report,name) for name in (
        "termination","iterations","usable","converged","initial_cost","final_cost","solve_seconds")}
    result["boundary_inclusive_seconds"] = elapsed
    result["objective_checked"] = objective(x,source,target,scale)
    result["translation_error_m"] = float(np.linalg.norm(x[:3]-TRUTH[:3]))
    result["rotation_error_rad"] = float((Rotation.from_rotvec(x[3:])*Rotation.from_rotvec(TRUTH[3:]).inv()).magnitude())
    result["x"] = x.tolist()
    return x,result

def scipy_solve(source,target,scale):
    start = time.perf_counter()
    report = least_squares(grouped_huber_residual,np.zeros(6),args=(source,target,scale),
                           loss="linear",method="trf",jac="3-point",x_scale=1.0,
                           ftol=TOLERANCE,xtol=TOLERANCE,gtol=TOLERANCE,max_nfev=100)
    return report.x, dict(seconds=time.perf_counter()-start,success=bool(report.success),
                          status=int(report.status),nfev=int(report.nfev),cost=float(report.cost),
                          objective_checked=objective(report.x,source,target,scale),
                          x=report.x.tolist())

def run(mode="compiled"):
    api,timing = load(mode)
    cases = {}
    for name,noise,outliers,scale in (("exact",0,False,0),("noise",0.005,False,0),
                                     ("outliers_linear",0.005,True,0),
                                     ("outliers_huber",0.005,True,HUBER_SCALE_M)):
        source,target=observations(noise,outliers)
        x,native=native_solve(api,source,target,scale)
        sx,scipy=scipy_solve(source,target,scale)
        cases[name]=dict(native=native,scipy=scipy,parameter_difference=float(np.linalg.norm(x-sx)))
    # Warm calls omit Python list construction. Inputs remain owning NumPy arrays.
    source,target=observations(0.005,True)
    durations=[]
    native_solve(api,source,target,HUBER_SCALE_M)
    for _ in range(20):
        _,report=native_solve(api,source,target,HUBER_SCALE_M)
        durations.append(report["boundary_inclusive_seconds"])
    timing["warm_native_median_seconds"]=float(np.median(durations))
    p,q=source[0].copy(),target[0].copy()
    x=np.zeros(6); r=np.empty(3); J=np.empty(18)
    api.evaluate(p,q,x,r,J)
    start=time.perf_counter()
    for _ in range(1000): api.evaluate(p,q,x,r,J)
    timing["warm_single_evaluation_boundary_seconds"]=(time.perf_counter()-start)/1000
    start=time.perf_counter()
    for _ in range(100): np.ascontiguousarray(source[:,::-1])
    timing["explicit_noncontiguous_copy_seconds"]=(time.perf_counter()-start)/100
    versions = {}
    for package in ("ceres-solver","manif","eigen","gcc_linux-64","gxx_linux-64","libgcc","libstdcxx","python","numpy","scipy","cppyy","cppyy-kit"):
        records=list((Path(__import__('sys').prefix)/"conda-meta").glob(package+"-*.json"))
        records=[p for p in records if json.loads(p.read_text())["name"]==package]
        versions[package]=[{k:json.loads(p.read_text())[k] for k in ("version","build")} for p in records]
    toolchain=np.zeros(6,dtype=np.int32); api.toolchain(toolchain)
    output=dict(cases=cases,timings=timing,versions=versions,toolchain=toolchain.tolist(),
                truth=TRUTH.tolist(),count=len(source),seed=2026,huber_scale_m=HUBER_SCALE_M)
    (ROOT/"build").mkdir(exist_ok=True)
    (ROOT/("build/results_"+mode+".json")).write_text(json.dumps(output,indent=2)+"\n")
    print(json.dumps(output,indent=2))
    return output

if __name__=="__main__": run("jit" if "--jit" in __import__('sys').argv else "compiled")
