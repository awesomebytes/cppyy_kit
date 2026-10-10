"""Independent acceptance check for a supplied count_speed_above module."""
import argparse
from dataclasses import replace
import importlib.util
from pathlib import Path
import numpy as np
from extract import generate_fixture,extract_poses


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("module",type=Path);args=parser.parse_args()
    spec=importlib.util.spec_from_file_location("candidate_query",args.module)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    path=Path(__file__).parent/"build/query-check.mcap";generate_fixture(path)
    batch=extract_poses(path)
    for threshold in [0.,.1,.3,1.,10.]:
        dt=np.diff(batch.log_time_ns).astype(float)*1e-9
        speed=np.linalg.norm(np.diff(batch.position_m,axis=0),axis=1)/dt
        assert module.count_speed_above(batch,threshold)==int(np.count_nonzero(speed>threshold))
    duplicate=replace(batch,log_time_ns=np.array([1,1,1_000_000_001],np.uint64),position_m=np.array([[0,0,0],[5,0,0],[6,0,0]],float))
    assert module.count_speed_above(duplicate,.5)==1
    empty=replace(batch,log_time_ns=np.empty(0,np.uint64),position_m=np.empty((0,3)))
    assert module.count_speed_above(empty)==0
    timestamps=np.ndarray(duplicate.log_time_ns.shape,dtype=np.uint64,buffer=bytearray(duplicate.log_time_ns.nbytes+1),offset=1)
    positions=np.ndarray(duplicate.position_m.shape,dtype=np.float64,buffer=bytearray(duplicate.position_m.nbytes+1),offset=1)
    timestamps[:]=duplicate.log_time_ns;positions[:]=duplicate.position_m
    assert module.count_speed_above(replace(duplicate,log_time_ns=timestamps,position_m=positions),.5)==1
    extreme=replace(duplicate,log_time_ns=np.array([0,10**9],np.uint64),position_m=np.array([[0,0,0],[1e200,0,0]],float))
    assert module.count_speed_above(extreme,2e200)==0
    for altered,threshold in [(replace(duplicate,log_time_ns=np.array([3,2,4],np.uint64)),.3),
                              (replace(duplicate,position_m=np.zeros((3,2))),.3),
                              (replace(duplicate,position_m=np.full((3,3),np.nan)),.3),(duplicate,-1.),(duplicate,np.nan)]:
        try:module.count_speed_above(altered,threshold)
        except Exception:pass
        else:raise AssertionError("invalid query input accepted")
    print("query acceptance passed")


if __name__=="__main__":main()
