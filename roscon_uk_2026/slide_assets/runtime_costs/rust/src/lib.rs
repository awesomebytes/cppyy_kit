use nalgebra::{SMatrix, SVector};
use numpy::{PyReadonlyArray1, PyReadonlyArray2, PyReadwriteArray1, PyUntypedArrayMethods};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

#[pyfunction]
fn joint_step(
    jacobian: PyReadonlyArray2<'_, f64>,
    delta: PyReadonlyArray1<'_, f64>,
    damping: f64,
    mut output: PyReadwriteArray1<'_, f64>,
) -> PyResult<()> {
    if jacobian.shape() != [6, 7] || delta.shape() != [6] || output.shape() != [7] {
        return Err(PyValueError::new_err("Expected shapes (6, 7), (6,), and (7,)"));
    }
    if !jacobian.is_c_contiguous() {
        return Err(PyValueError::new_err("Jacobian must be C-contiguous"));
    }
    let j = SMatrix::<f64, 6, 7>::from_row_slice(jacobian.as_slice()?);
    let dx = SVector::<f64, 6>::from_row_slice(delta.as_slice()?);
    let a = j * j.transpose() + SMatrix::<f64, 6, 6>::identity() * damping.powi(2);
    let factor = a.cholesky().ok_or_else(|| {
        PyValueError::new_err("Damped matrix must be positive definite")
    })?;
    let dq = j.transpose() * factor.solve(&dx);
    output.as_slice_mut()?.copy_from_slice(dq.as_slice());
    Ok(())
}

#[pymodule]
fn joint_step_rust(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_function(wrap_pyfunction!(joint_step, module)?)?;
    module.add_function(wrap_pyfunction!(_transform_addresses, module)?)?;
    Ok(())
}

// Private benchmark entry point, called only through the shared Python adapter.
// The adapter validates dtype, shape, alignment, disjoint output and lifetime.
// The GIL stays held. Buffers must not be resized or altered concurrently.
#[pyfunction]
fn _transform_addresses(source: usize, transform: usize, destination: usize, count: usize) {
    unsafe {
        let points = std::slice::from_raw_parts(source as *const f64, count * 3);
        let t = std::slice::from_raw_parts(transform as *const f64, 12);
        let out = std::slice::from_raw_parts_mut(destination as *mut f64, count * 4);
        for i in 0..count {
            let x = points[3*i];
            let y = points[3*i+1];
            let z = points[3*i+2];
            let a = t[0]*x + t[1]*y + t[2]*z + t[3];
            let b = t[4]*x + t[5]*y + t[6]*z + t[7];
            let c = t[8]*x + t[9]*y + t[10]*z + t[11];
            out[4*i] = a;
            out[4*i+1] = b;
            out[4*i+2] = c;
            out[4*i+3] = a*a + b*b + c*c;
        }
    }
}
