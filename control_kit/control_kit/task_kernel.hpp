#pragma once
#include <Eigen/Geometry>
#include <Eigen/Cholesky>
namespace control_kit_tasks {
// Rows are parent index, coordinate index and type (0 fixed, 1 rotation, 2 translation).
void fk(int count, const int* tree, const double* origins, const double* axes,
        const double* q, double* output) {
  using namespace Eigen;
  Isometry3d base = Isometry3d::Identity();
  base.translation() = Vector3d(q[0], q[1], 0);
  base.linear() = AngleAxisd(q[2], Vector3d::UnitZ()).toRotationMatrix();
  for (int i=0; i<count; ++i) {
    int parent=tree[3*i], coordinate=tree[3*i+1], type=tree[3*i+2];
    Isometry3d t=Isometry3d::Identity();
    t.translation()=Map<const Vector3d>(origins+6*i);
    t.linear()=(AngleAxisd(origins[6*i+5],Vector3d::UnitZ()) *
                AngleAxisd(origins[6*i+4],Vector3d::UnitY()) *
                AngleAxisd(origins[6*i+3],Vector3d::UnitX())).toRotationMatrix();
    if (type==1) t.rotate(AngleAxisd(q[coordinate],Map<const Vector3d>(axes+3*i)));
    if (type==2) t.translate(q[coordinate]*Map<const Vector3d>(axes+3*i));
    Matrix4d p=base.matrix();
    if (parent>=0) p=Map<const Matrix<double,4,4,RowMajor>>(output+16*parent);
    Map<Matrix<double,4,4,RowMajor>>(output+16*i)=p*t.matrix();
  }
}
// Joint cost is the squared cost of unit joint speed. Rows already contain task weights.
void solve(int rows,int cols,const double* jacobian,const double* error,
           const double* cost,double damping,double* velocity) {
  using namespace Eigen;
  Map<const Matrix<double,Dynamic,Dynamic,RowMajor>> j(jacobian,rows,cols);
  Map<const VectorXd> e(error,rows);
  MatrixXd h=j.transpose()*j;
  for(int i=0;i<cols;++i) h(i,i)+=damping*cost[i];
  Map<VectorXd>(velocity,cols)=h.ldlt().solve(j.transpose()*e);
}
}
