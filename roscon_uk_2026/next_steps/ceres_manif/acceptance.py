"""Independent SciPy/NumPy references for the native calibration contract."""
import json
import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation
from calibration import (TRUTH, ROOT, HUBER_SCALE_M, native_solve, observations,
                         objective, raw_residual, scipy_solve)
from native import load

def assert_rejected(fn,needle):
    try:
        fn()
    except Exception as error:
        assert needle in str(error),str(error)
    else:
        raise AssertionError("expected rejection: "+needle)

def main(mode="compiled"):
    api,_=load(mode)
    step=1e-6
    p=np.array([0.4,-0.8,1.2]); q=np.array([-0.1,0.9,0.3])
    errors=[]
    # Include the small-angle branch, a generic pose, and larger rotations.
    for x in (np.zeros(6),TRUTH.copy(),np.array([0.5,-0.7,1.2,1.7,-0.2,0.8])):
        r=np.empty(3); jac=np.empty(18)
        api.evaluate(p,q,x,r,jac)
        expected=raw_residual(x,p[None,:],q[None,:]).ravel()
        np.testing.assert_allclose(r,expected,atol=1e-12)
        reference=np.column_stack([
            (raw_residual(x+step*np.eye(6)[j],p[None,:],q[None,:])-
             raw_residual(x-step*np.eye(6)[j],p[None,:],q[None,:])).ravel()/(2*step)
            for j in range(6)])
        errors.append(float(np.max(np.abs(jac.reshape(3,6)-reference))))
        np.testing.assert_allclose(jac.reshape(3,6),reference,atol=2e-8,rtol=2e-8)
    # manif's act Jacobian is in right SE(3) tangent coordinates, unlike x above.
    x=TRUTH.copy(); output=np.empty(3); jac=np.empty(18); delta=np.zeros(6)
    api.group_action(x,p,delta,output,jac)
    R=Rotation.from_rotvec(x[3:]).as_matrix()
    def independently_perturbed(d):
        wx,wy,wz=d[3:]
        twist=np.zeros((4,4))
        twist[:3,:3]=[[0,-wz,wy],[wz,0,-wx],[-wy,wx,0]]
        twist[:3,3]=d[:3]
        moved=(expm(twist)@np.r_[p,1])[:3]
        return R@moved+x[:3]
    reference=np.column_stack([(independently_perturbed(step*np.eye(6)[j])-
                                independently_perturbed(-step*np.eye(6)[j]))/(2*step)
                               for j in range(6)])
    np.testing.assert_allclose(jac.reshape(3,6),reference,atol=2e-8,rtol=2e-8)
    delta=np.array([0.04,-0.02,0.03,0.08,0.03,-0.05])
    api.group_action(x,p,delta,output,jac)
    np.testing.assert_allclose(output,independently_perturbed(delta),atol=1e-12)
    # Closed-form Kabsch gives a second exact rigid-registration reference.
    source,target=observations()
    P=source-source.mean(axis=0); Q=target-target.mean(axis=0)
    U,_,Vt=np.linalg.svd(P.T@Q)
    rotation=Vt.T@np.diag([1,1,np.linalg.det(Vt.T@U.T)])@U.T
    translation=target.mean(axis=0)-rotation@source.mean(axis=0)
    x,report=native_solve(api,source,target)
    assert report["converged"] and report["usable"]
    np.testing.assert_allclose(x[:3],translation,atol=1e-9)
    np.testing.assert_allclose(Rotation.from_rotvec(x[3:]).as_matrix(),rotation,atol=1e-9)
    assert report["translation_error_m"]<1e-9 and report["rotation_error_rad"]<1e-9
    cases={}
    for name,noise,outliers,scale in (("noise",0.005,False,0),
                                     ("outliers_huber",0.005,True,HUBER_SCALE_M)):
        source,target=observations(noise,outliers)
        x,report=native_solve(api,source,target,scale)
        scipy_x,scipy_report=scipy_solve(source,target,scale)
        assert report["converged"] and scipy_report["success"]
        assert report["translation_error_m"]<0.004
        assert report["rotation_error_rad"]<0.006
        assert abs(report["final_cost"]-objective(x,source,target,scale))<1e-12
        assert abs(report["final_cost"]-scipy_report["cost"])<1e-10
        np.testing.assert_allclose(x,scipy_x,atol=2e-7,rtol=0)
        cases[name]=report
    source,target=observations(0.005,True)
    _,linear=native_solve(api,source,target,0)
    assert cases["outliers_huber"]["translation_error_m"]<linear["translation_error_m"]
    # One vector loss per correspondence: coordinatewise SciPy Huber is different.
    residual=np.array([0.04,0.04,0.0]); a=HUBER_SCALE_M
    grouped=0.5*(2*a*np.linalg.norm(residual)-a*a)
    coordinatewise=0.5*np.sum(np.where(residual**2<=a*a,residual**2,2*a*np.abs(residual)-a*a))
    assert abs(grouped-coordinatewise)>1e-4
    _,failed=native_solve(api,source,target,HUBER_SCALE_M,max_iterations=0)
    assert not failed["converged"] and failed["termination"]==1
    line=np.ascontiguousarray(np.column_stack((np.arange(10.),np.zeros(10),np.zeros(10))))
    assert_rejected(lambda:native_solve(api,line,line),"degenerate source")
    repeated=np.ones((10,3))
    assert_rejected(lambda:native_solve(api,repeated,repeated),"degenerate source")
    planar=observations()[0].copy(); planar[:,2]=0
    planar_target=np.ascontiguousarray(Rotation.from_rotvec(TRUTH[3:]).apply(planar)+TRUTH[:3])
    _,planar_report=native_solve(api,planar,planar_target)
    assert planar_report["converged"] and planar_report["translation_error_m"]<1e-9
    assert_rejected(lambda:native_solve(api,source,target[:-1]),"invalid shape")
    assert_rejected(lambda:native_solve(api,source[:,::-1],target),"C-contiguous")
    invalid=source.copy(); invalid[0,0]=np.nan
    assert_rejected(lambda:native_solve(api,invalid,target),"finite")
    assert_rejected(lambda:native_solve(api,source,target,-1),"invalid solve")
    result=dict(autodiff_max_absolute_errors=errors,
                manif_right_jacobian_max_absolute_error=float(np.max(np.abs(jac.reshape(3,6)-reference))),
                recovery=cases,failed_solver=failed,checks_passed=True)
    (ROOT/("build/acceptance_"+mode+".json")).write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))

if __name__=="__main__": main("jit" if "--jit" in __import__('sys').argv else "compiled")
