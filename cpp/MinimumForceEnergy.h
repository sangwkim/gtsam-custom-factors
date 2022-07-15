#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

// n, c, o, g, s
class MinimumForceEnergy: public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6> {

private:

  double epsilon_;
  int ver_;

public:

  MinimumForceEnergy(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, int Version, double cost_sigma, double eps) :
      gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6>(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), key1, key2, key3, key4, key5),
      epsilon_(eps), ver_(Version) {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3& n,
    const gtsam::Pose3& c,
    const gtsam::Pose3& o,
    const gtsam::Pose3& g,
    const gtsam::Vector6& s,
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
      gtsam::Vector s_ = (gtsam::Vector6() << s[0], pow(s[1],-0.5)*s[0], pow(s[2],-0.5)*s[0], pow(s[3],-0.5)*s[5], pow(s[4],-0.5)*s[5], s[5]).finished();
      gtsam::Matrix66 Hss = (gtsam::Matrix66() << 1, 0, 0, 0, 0, 0,
                                                  pow(s[1],-0.5), -0.5*pow(s[1],-1.5)*s[0], 0, 0, 0, 0,
                                                  pow(s[2],-0.5), 0, -0.5*pow(s[2],-1.5)*s[0], 0, 0, 0,
                                                  0, 0, 0, -0.5*pow(s[3],-1.5)*s[5], 0, pow(s[3],-0.5),
                                                  0, 0, 0, 0, -0.5*pow(s[4],-1.5)*s[5], pow(s[4],-0.5),
                                                  0, 0, 0, 0, 0, 1).finished();
      
      // Compute penetration distance
      gtsam::Pose3 oc = gtsam::traits<gtsam::Pose3>::Between(o, c, &Ho1, &Hc1);
      gtsam::Pose3 nc = gtsam::traits<gtsam::Pose3>::Between(n, c, &Hn2, &Hc2);
      gtsam::Pose3 ncoc = gtsam::traits<gtsam::Pose3>::Between(nc, oc, &Hnc, &Hoc);
      gtsam::Point3 ncoct = ncoc.translation(&Hncoc);
      double pen_dist = - ncoct.z();
      gtsam::Matrix13 Hpd = (gtsam::Matrix13() << 0, 0, -1).finished();

      gtsam::Vector3 s_trn = (gtsam::Vector3() << s_[3], s_[4], s_[5]).finished();
      gtsam::Vector3 s_rot = (gtsam::Vector3() << s_[0], s_[1], s_[2]).finished();
      gtsam::Matrix36 Htrn = (gtsam::Matrix36() << 0, 0, 0, 1, 0, 0,
                                                   0, 0, 0, 0, 1, 0,
                                                   0, 0, 0, 0, 0, 1).finished();
      gtsam::Matrix36 Hrot = (gtsam::Matrix36() << 1, 0, 0, 0, 0, 0,
                                                   0, 1, 0, 0, 0, 0,
                                                   0, 0, 1, 0, 0, 0).finished();
      gtsam::Pose3 gc = gtsam::traits<gtsam::Pose3>::Between(g, c, &Hg3, &Hc3);
      gtsam::Rot3 R_gc = gc.rotation(&HgcR);
      
      // Compute effective translational stiffness
      gtsam::Point3 u = R_gc.rotate((gtsam::Point3() << 0, 0, 1).finished(), &Hgc12, &Hdum);
      double s_trn_c_z_squared = pow(u.x(),2) * pow(s_trn[0],2) + pow(u.y(),2) * pow(s_trn[1],2) + pow(u.z(),2) * pow(s_trn[2],2);
      gtsam::Matrix13 Hu13 = (gtsam::Matrix13() << 2*u.x()*pow(s_trn[0],2), 2*u.y()*pow(s_trn[1],2), 2*u.z()*pow(s_trn[2],2)).finished();
      gtsam::Matrix13 Hstrn13 = (gtsam::Matrix13() << pow(u.x(),2)*2*s_trn[0], pow(u.y(),2)*2*s_trn[1], pow(u.z(),2)*2*s_trn[2]).finished();

      // Compute effective rotational stiffness
      gtsam::Point3 t_gc = gc.translation(&Hgc6);
      gtsam::Point3 v = gtsam::cross(t_gc, u, &Htgc14, &Hu14);
      double s_rot_eff_sn = pow(v.x(),2) * pow(s_rot[0],2) + pow(v.y(),2) * pow(s_rot[1],2) + pow(v.z(),2) * pow(s_rot[2],2);
      gtsam::Matrix13 Hv15 = (gtsam::Matrix13() << 2*v.x()*pow(s_rot[0],2), 2*v.y()*pow(s_rot[1],2), 2*v.z()*pow(s_rot[2],2)).finished();
      gtsam::Matrix13 Hsrot15 = (gtsam::Matrix13() << pow(v.x(),2)*2*s_rot[0], pow(v.y(),2)*2*s_rot[1], pow(v.z(),2)*2*s_rot[2]).finished();

      double s_total_eff_sn = s_trn_c_z_squared + s_rot_eff_sn;

      // Compute penetration distance due to translation
      gtsam::Pose3 og = gtsam::traits<gtsam::Pose3>::Between(o, g, &Ho9, &Hg9);
      gtsam::Pose3 ng = gtsam::traits<gtsam::Pose3>::Between(n, g, &Hn10, &Hg10);
      gtsam::Pose3 ngog = gtsam::traits<gtsam::Pose3>::Between(ng, og, &Hng, &Hog);
      gtsam::Point3 ngogt = ngog.translation(&Hngog);
      gtsam::Point3 ngogtc = R_gc.unrotate(ngogt, &Hgc11, &Hngogt);
      double pen_dist_trn = - ngogtc.z();
      
      double s_total_eff = pow(s_total_eff_sn, 0.5);

      if (ver_==0) { // energy mode
        if (pen_dist > s_total_eff * pow(epsilon_,0.5)) {
          if (Hn) *Hn = gtsam::Matrix16::Zero();
          if (Hc) *Hc = gtsam::Matrix16::Zero();
          if (Ho) *Ho = gtsam::Matrix16::Zero();
          if (Hg) *Hg = gtsam::Matrix16::Zero();
          if (Hs) *Hs = gtsam::Matrix16::Zero();
          return (gtsam::Vector1() << 0.0).finished();
        } else {
          //if (Hn) *Hn = - Hpd * Hncoc * Hnc * Hn2;
          //if (Hc) *Hc = ((pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hnorm * Hsrotc* Hproj* Hgc5 + pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hsq * Hz * Hgc4 + pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hnorm * Htgc8 * Hproj * Hgc7) * HgcR + pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hnorm * Htgc8 * Hproj * Htgc7 * Hgc6) * Hc3 - Hpd * Hncoc * Hnc * Hc2 - Hpd * Hncoc * Hoc * Hc1;
          //if (Ho) *Ho = - Hpd * Hncoc * Hoc * Ho1;
          //if (Hg) *Hg = ((pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hnorm * Hsrotc* Hproj* Hgc5 + pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hsq * Hz * Hgc4 + pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hnorm * Htgc8 * Hproj * Hgc7) * HgcR + pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hnorm * Htgc8 * Hproj * Htgc7 * Hgc6) * Hg3;
          //if (Hs) *Hs = (pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hsq * Hz * Hstrn * Htrn + pow(epsilon_,0.5) * 0.5 * pow(s_total_eff_sn,-0.5) * Hnorm * Hsrotc * Hproj * Hsrot * Hrot) * Hss;
          return (gtsam::Vector1() << s_total_eff * pow(epsilon_,0.5) - pen_dist).finished();
        }
      } else if (ver_==1) {
        if (pen_dist_trn > s_trn_c_z_squared * epsilon_) {
          if (Hn) *Hn = gtsam::Matrix16::Zero();
          if (Hc) *Hc = gtsam::Matrix16::Zero();
          if (Ho) *Ho = gtsam::Matrix16::Zero();
          if (Hg) *Hg = gtsam::Matrix16::Zero();
          if (Hs) *Hs = gtsam::Matrix16::Zero();
          return (gtsam::Vector1() << 0.0).finished();
        } else {
          if (Hn) *Hn = - Hpd*Hngogt*Hngog*Hng*Hn10;
          if (Hc) *Hc = epsilon_*Hu13*Hgc12*HgcR*Hc3;
          if (Ho) *Ho = - Hpd*Hngogt*Hngog*Hog*Ho9;
          if (Hg) *Hg = epsilon_*Hu13*Hgc12*HgcR*Hg3 - Hpd*Hngogt*Hngog*Hog*Hg9 - Hpd*Hngogt*Hngog*Hng*Hg10;
          //if (Hs) *Hs = epsilon_*Hstrn13*Htrn*Hss;
          if (Hs) *Hs = gtsam::Matrix16::Zero();
          return (gtsam::Vector1() << s_trn_c_z_squared * epsilon_ - pen_dist_trn).finished();
        }
      } else if (ver_==2) {
        if (pen_dist > s_total_eff_sn * epsilon_) {
          if (Hn) *Hn = gtsam::Matrix16::Zero();
          if (Hc) *Hc = gtsam::Matrix16::Zero();
          if (Ho) *Ho = gtsam::Matrix16::Zero();
          if (Hg) *Hg = gtsam::Matrix16::Zero();
          if (Hs) *Hs = gtsam::Matrix16::Zero();
          return (gtsam::Vector1() << 0.0).finished();
        } else {
          if (Hn) *Hn = - Hpd*Hncoc*Hnc*Hn2;
          if (Hc) *Hc = ((epsilon_*Hu13 + epsilon_*Hv15*Hu14)*Hgc12*HgcR + epsilon_*Hv15*Htgc14*Hgc6)*Hc3 - Hpd*Hncoc*Hoc*Hc1 - Hpd*Hncoc*Hnc*Hc2;
          if (Ho) *Ho = - Hpd*Hncoc*Hoc*Ho1;
          if (Hg) *Hg = ((epsilon_*Hu13 + epsilon_*Hv15*Hu14)*Hgc12*HgcR + epsilon_*Hv15*Htgc14*Hgc6)*Hg3;
          //if (Hs) *Hs = (epsilon_*Hstrn13*Htrn + epsilon_*Hsrot15*Hrot)*Hss;
          if (Hs) *Hs = gtsam::Matrix16::Zero();
          return (gtsam::Vector1() << s_total_eff_sn * epsilon_ - pen_dist).finished();
        }
      }

      
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 5;
  }

}; // \class MinimumForceEnergy

} /// namespace gtsam_packing