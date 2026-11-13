#pragma once
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Reference-only flatness feedforward. Forces use world axes; rates use the
// desired body axes. The existing position-feedback attitude remains in use.
namespace lab4ff {
inline Eigen::Vector3d veeSkew(const Eigen::Matrix3d& a) {
  return Eigen::Vector3d(a(2,1)-a(1,2), a(0,2)-a(2,0), a(1,0)-a(0,1))*0.5;
}
inline bool normalizeDerivatives(const Eigen::Vector3d& v,
    const Eigen::Vector3d& dv, const Eigen::Vector3d& ddv,
    Eigen::Vector3d* u, Eigen::Vector3d* du, Eigen::Vector3d* ddu) {
  const double n = v.norm();
  if (!v.allFinite() || !dv.allFinite() || !ddv.allFinite() || n < 1e-4) return false;
  *u = v/n;
  const double dn = u->dot(dv);
  *du = (dv-(*u)*dn)/n;
  const double ddn = du->dot(dv)+u->dot(ddv);
  *ddu = ddv/n - 2.0*dv*dn/(n*n) - (*u)*ddn/n + 2.0*(*u)*dn*dn/(n*n);
  return u->allFinite() && du->allFinite() && ddu->allFinite();
}
inline bool compute(const Eigen::Vector3d& a, const Eigen::Vector3d& jerk,
    const Eigen::Vector3d& snap, double yaw, double dyaw, double ddyaw,
    Eigen::Matrix3d* R, Eigen::Vector3d* omega, Eigen::Vector3d* alpha) {
  if (!std::isfinite(yaw) || !std::isfinite(dyaw) || !std::isfinite(ddyaw)) return false;
  Eigen::Vector3d b3, db3, ddb3, b2, db2, ddb2;
  if (!normalizeDerivatives(a+Eigen::Vector3d(0,0,9.81),jerk,snap,&b3,&db3,&ddb3)) return false;
  const Eigen::Vector3d xc(std::cos(yaw),std::sin(yaw),0);
  const Eigen::Vector3d dx = dyaw*Eigen::Vector3d(-std::sin(yaw),std::cos(yaw),0);
  const Eigen::Vector3d ddx = ddyaw*Eigen::Vector3d(-std::sin(yaw),std::cos(yaw),0)-dyaw*dyaw*xc;
  const Eigen::Vector3d c=b3.cross(xc);
  const Eigen::Vector3d dc=db3.cross(xc)+b3.cross(dx);
  const Eigen::Vector3d ddc=ddb3.cross(xc)+2.0*db3.cross(dx)+b3.cross(ddx);
  if (!normalizeDerivatives(c,dc,ddc,&b2,&db2,&ddb2)) return false;
  const Eigen::Vector3d b1=b2.cross(b3);
  const Eigen::Vector3d db1=db2.cross(b3)+b2.cross(db3);
  const Eigen::Vector3d ddb1=ddb2.cross(b3)+2.0*db2.cross(db3)+b2.cross(ddb3);
  Eigen::Matrix3d dR, ddR;
  R->col(0)=b1; R->col(1)=b2; R->col(2)=b3;
  dR.col(0)=db1; dR.col(1)=db2; dR.col(2)=db3;
  ddR.col(0)=ddb1; ddR.col(1)=ddb2; ddR.col(2)=ddb3;
  *omega=veeSkew(R->transpose()*dR);
  *alpha=veeSkew(dR.transpose()*dR+R->transpose()*ddR);
  return R->allFinite() && omega->allFinite() && alpha->allFinite();
}
}  // namespace lab4ff
