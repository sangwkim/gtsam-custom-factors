#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

// n, c, o, g, s
class MinimumForce: public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector9> {

private:

  double epsilon_, d_opt_;

public:

  MinimumForce(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, double cost_sigma, double eps, double d_opt) :
      gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector9>(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), key1, key2, key3, key4, key5),
      epsilon_(eps), d_opt_(d_opt) {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3& n,
    const gtsam::Pose3& c,
    const gtsam::Pose3& o,
    const gtsam::Pose3& g,
    const gtsam::Vector9& s,
    boost::optional<gtsam::Matrix&> Hn = boost::none,
    boost::optional<gtsam::Matrix&> Hc = boost::none,
    boost::optional<gtsam::Matrix&> Ho = boost::none,
    boost::optional<gtsam::Matrix&> Hg = boost::none,
    boost::optional<gtsam::Matrix&> Hs = boost::none
    ) const {

      // Define Jacobians
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Ho1, Hc1, Hn2, Hc2, Hnc, Hoc, Hg3, Hc3, Ho9, Hg9, Hn10, Hg10, Hng, Hog;
      gtsam::Matrix36 Hncoc, HgcR, Hgc6, Hngog;
      gtsam::Matrix33 Hgc11, Hngogt, Hgc12, Htgc14, Hu14, Hdum;
      
      // Convert stiffness
      gtsam::Vector s_ = (gtsam::Vector6() << s[0], s[1], s[2], s[3], s[4], s[5]).finished(); //(gtsam::Vector6() << s[0], pow(s[1],-0.5)*s[0], pow(s[2],-0.5)*s[0], pow(s[3],-0.5)*s[5], pow(s[4],-0.5)*s[5], s[5]).finished();
      
      // Compute penetration distance
      gtsam::Pose3 oc = gtsam::traits<gtsam::Pose3>::Between(o, c, &Ho1, &Hc1);
      gtsam::Pose3 nc = gtsam::traits<gtsam::Pose3>::Between(n, c, &Hn2, &Hc2);
      gtsam::Pose3 ncoc = gtsam::traits<gtsam::Pose3>::Between(nc, oc, &Hnc, &Hoc);
      gtsam::Point3 ncoct = ncoc.translation(&Hncoc);
      double pen_dist = - ncoct.z();
      gtsam::Matrix13 Hpd = (gtsam::Matrix13() << 0, 0, -1).finished();

      gtsam::Vector3 s_trn = (gtsam::Vector3() << s_[3], s_[4], s_[5]).finished();
      gtsam::Vector3 s_rot = (gtsam::Vector3() << s_[0], s_[1], s_[2]).finished();
      gtsam::Pose3 gc = gtsam::traits<gtsam::Pose3>::Between(g, c, &Hg3, &Hc3);
      gtsam::Rot3 R_gc = gc.rotation(&HgcR);
      
      // Compute effective translational stiffness
      gtsam::Point3 u = R_gc.rotate((gtsam::Point3() << 0, 0, 1).finished(), &Hgc12, &Hdum);
      double s_trn_c_z_squared = pow(u.x(),2) * pow(s_trn[0],2) + pow(u.y(),2) * pow(s_trn[1],2) + pow(u.z(),2) * pow(s_trn[2],2);

      // Compute effective rotational stiffness
      gtsam::Point3 t_gc = gc.translation(&Hgc6);
      gtsam::Point3 t_gc_ = (gtsam::Vector3() << t_gc.x()-s[6], t_gc.y()-s[7], t_gc.z()-s[8]).finished();

      gtsam::Point3 v = gtsam::cross(t_gc_, u, &Htgc14, &Hu14);
      double s_rot_eff_sn = pow(v.x(),2) * pow(s_rot[0],2) + pow(v.y(),2) * pow(s_rot[1],2) + pow(v.z(),2) * pow(s_rot[2],2);
      
      double s_total_eff_sn = s_trn_c_z_squared + s_rot_eff_sn / d_opt_;
      
      double s_total_eff = pow(s_total_eff_sn, 0.5);

      if (pen_dist > s_total_eff_sn * epsilon_) {
        if (Hn) *Hn = gtsam::Matrix16::Zero();
        if (Hc) *Hc = gtsam::Matrix16::Zero();
        if (Ho) *Ho = gtsam::Matrix16::Zero();
        if (Hg) *Hg = gtsam::Matrix16::Zero();
        if (Hs) *Hs = gtsam::Matrix19::Zero();
        return (gtsam::Vector1() << 0.0).finished();
      } else {
        if (Hn) *Hn = - Hpd*Hncoc*Hnc*Hn2;
        if (Hc) *Hc = - Hpd*Hncoc*Hoc*Hc1 - Hpd*Hncoc*Hnc*Hc2;
        if (Ho) *Ho = - Hpd*Hncoc*Hoc*Ho1;
        if (Hg) *Hg = gtsam::Matrix16::Zero();
        if (Hs) *Hs = gtsam::Matrix19::Zero();
        return (gtsam::Vector1() << s_total_eff_sn * epsilon_ - pen_dist).finished();
      }

      
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 5;
  }

}; // \class MinimumForce

} /// namespace gtsam_packing