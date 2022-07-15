#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

// n, c, o, g, s
class TangentialForce: public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6> {


public:

  TangentialForce(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, double cost_sigma) :
      gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6>(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), key1, key2, key3, key4, key5){}

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
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Ho1, Hg1, Hn2, Hg2, Hng, Hog, Hg3, Hc3;
      gtsam::Matrix36 Hngog, Hgc;
      gtsam::Matrix33 HRgc, Hf;

      // Convert stiffness
      gtsam::Vector s_ = (gtsam::Vector6() << s[0], pow(s[1],-0.5)*s[0], pow(s[2],-0.5)*s[0], pow(s[3],-0.5)*s[5], pow(s[4],-0.5)*s[5], s[5]).finished();
      gtsam::Matrix66 Hss = (gtsam::Matrix66() << 1, 0, 0, 0, 0, 0,
                                                  pow(s[1],-0.5), -0.5*pow(s[1],-1.5)*s[0], 0, 0, 0, 0,
                                                  pow(s[2],-0.5), 0, -0.5*pow(s[2],-1.5)*s[0], 0, 0, 0,
                                                  0, 0, 0, -0.5*pow(s[3],-1.5)*s[5], 0, pow(s[3],-0.5),
                                                  0, 0, 0, 0, -0.5*pow(s[4],-1.5)*s[5], pow(s[4],-0.5),
                                                  0, 0, 0, 0, 0, 1).finished();

      gtsam::Pose3 og = gtsam::traits<gtsam::Pose3>::Between(o, g, &Ho1, &Hg1);
      gtsam::Pose3 ng = gtsam::traits<gtsam::Pose3>::Between(n, g, &Hn2, &Hg2);
      gtsam::Pose3 ngog = gtsam::traits<gtsam::Pose3>::Between(ng, og, &Hng, &Hog);
      gtsam::Point3 ngogt = ngog.translation(&Hngog);
      gtsam::Vector3 force = (gtsam::Vector3() << ngogt.x()*pow(s_[3],-2), ngogt.y()*pow(s_[4],-2), ngogt.z()*pow(s_[5],-2)).finished();
      gtsam::Matrix Hngogt = (gtsam::Matrix33() << pow(s_[3],-2), 0, 0,
                                                   0, pow(s_[4],-2), 0,
                                                   0, 0, pow(s_[5],-2)).finished();
      gtsam::Matrix Hs_ = (gtsam::Matrix36() << 0, 0, 0, -2*ngogt.x()*pow(s_[3],-3), 0, 0,
                                                0, 0, 0, 0, -2*ngogt.y()*pow(s_[4],-3), 0,
                                                0, 0, 0, 0, 0, -2*ngogt.z()*pow(s_[5],-3)).finished();

      gtsam::Pose3 gc = gtsam::traits<gtsam::Pose3>::Between(g, c, &Hg3, &Hc3);
      gtsam::Rot3 R_gc = gc.rotation(&Hgc);

      gtsam::Point3 force_c = R_gc.unrotate(force, &HRgc, &Hf);
      double force_c_tan_sn = pow(force_c.x(),2) + pow(force_c.y(),2);
      gtsam::Matrix Hfc = (gtsam::Matrix13() << 2*force_c.x(), 2*force_c.y(), 0).finished();
      double force_c_tan_norm = pow(force_c_tan_sn, 0.5);
      double Hsn = 0.5 * pow(force_c_tan_sn, -0.5);

      if (Hn) *Hn = Hsn*Hfc*Hf*Hngogt*Hngog*Hng*Hn2;
      if (Hc) *Hc = Hsn*Hfc*HRgc*Hgc*Hc3;
      if (Ho) *Ho = Hsn*Hfc*Hf*Hngogt*Hngog*Hog*Ho1;
      if (Hg) *Hg = Hsn*Hfc*Hf*Hngogt*Hngog*Hog*Hg1 + Hsn*Hfc*Hf*Hngogt*Hngog*Hng*Hg2 + Hsn*Hfc*HRgc*Hgc*Hg3;
      //if (Hs) *Hs = Hsn*Hfc*Hf*Hs_*Hss;
      if (Hs) *Hs = gtsam::Matrix16::Zero();

      return (gtsam::Vector1() << force_c_tan_norm).finished();      
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 5;
  }

}; // \class TangentialForce

} /// namespace gtsam_packing