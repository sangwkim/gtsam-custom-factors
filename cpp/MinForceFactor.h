#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

class MinForceFactor: public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6> {
// n, o, g, c, v

private:
  double epsilon_;

public:

  MinForceFactor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, double cost_sigma, double eps) :
      gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6>(
        gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), key1, key2, key3, key4, key5), epsilon_(eps) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& n, const gtsam::Pose3& o,
    const gtsam::Pose3& g, const gtsam::Pose3& c, const gtsam::Vector6& v,
    boost::optional<gtsam::Matrix&> Hn = boost::none,
    boost::optional<gtsam::Matrix&> Ho = boost::none,
    boost::optional<gtsam::Matrix&> Hg = boost::none,
    boost::optional<gtsam::Matrix&> Hc = boost::none,
    boost::optional<gtsam::Matrix&> Hv = boost::none) const {

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hn1, Hn2, Ho1, Ho2, Hg1, Hg2, Hc1, Hc2, Hng, Hog, Hnc, Hoc, Hlm;
      gtsam::Pose3 ng = gtsam::traits<gtsam::Pose3>::Between(n,g,&Hn1,&Hg1);
      gtsam::Pose3 og = gtsam::traits<gtsam::Pose3>::Between(o,g,&Ho1,&Hg2);

      gtsam::Pose3 hx = gtsam::traits<gtsam::Pose3>::Between(ng,og,&Hng,&Hog);
      gtsam::Vector6 lm = gtsam::traits<gtsam::Pose3>::Logmap(hx, &Hlm);

      gtsam::Vector s = (gtsam::Vector6() << v[0], pow(v[1],-0.5)*v[0], pow(v[2],-0.5)*v[0], v[3], pow(v[4],-0.5)*v[3], pow(v[5],-0.5)*v[3]).finished();
      /*
      gtsam::Matrix66 Hsv = (gtsam::Matrix66() << 1, 0, 0, 0, 0, 0,
                                                  pow(v[1],-0.5), -0.5*pow(v[1],-1.5)*v[0], 0, 0, 0, 0,
                                                  pow(v[2],-0.5), 0, -0.5*pow(v[2],-1.5)*v[0], 0, 0, 0,
                                                  0, 0, 0, 1, 0, 0,
                                                  0, 0, 0, pow(v[4],-0.5), -0.5*pow(v[4],-1.5)*v[3], 0,
                                                  0, 0, 0, pow(v[4],-0.5), 0, -0.5*pow(v[5],-1.5)*v[3]).finished();
      */
      gtsam::Vector6 lm_ = gtsam::ediv_(lm, s);
      gtsam::Matrix66 Hlm_ = (gtsam::Matrix66() << 1/s[0], 0, 0, 0, 0, 0,
                                                   0, 1/s[1], 0, 0, 0, 0,
                                                   0, 0, 1/s[2], 0, 0, 0,
                                                   0, 0, 0, 1/s[3], 0, 0,
                                                   0, 0, 0, 0, 1/s[4], 0,
                                                   0, 0, 0, 0, 0, 1/s[5]).finished();

      double Energy = lm_.squaredNorm();
      gtsam::Matrix16 Hnorm = (gtsam::Matrix16() << 2*lm_[0], 2*lm_[1], 2*lm_[2], 2*lm_[3], 2*lm_[4], 2*lm_[5]).finished();


      gtsam::Pose3 nc = gtsam::traits<gtsam::Pose3>::Between(n,c,&Hn2,&Hc1);
      gtsam::Pose3 oc = gtsam::traits<gtsam::Pose3>::Between(o,c,&Ho2,&Hc2);
      gtsam::Pose3 ncoc = gtsam::traits<gtsam::Pose3>::Between(nc,oc,&Hnc,&Hoc);
      double z = - ncoc.translation().z();
      gtsam::Matrix16 Hz = (gtsam::Matrix16() << 0, 0, 0, 0, 0, -1).finished();

      if (Energy > epsilon_ * z) {
        if (Hn) *Hn = gtsam::Matrix16::Zero();
        if (Ho) *Ho = gtsam::Matrix16::Zero();
        if (Hg) *Hg = gtsam::Matrix16::Zero();
        if (Hc) *Hc = gtsam::Matrix16::Zero();
        if (Hv) *Hv = gtsam::Matrix16::Zero();
        return (gtsam::Vector1() << 0,0).finished();
      } else {
        if (Hn) *Hn = Hnorm * Hlm_ * Hlm * Hng * Hn1 - epsilon_ * Hz * Hnc * Hn2;
        //if (Ho) *Ho = Hnorm * Hlm_ * Hlm * Hog * Ho1 - epsilon_ * Hz * Hoc * Ho2;
        if (Ho) *Ho = gtsam::Matrix16::Zero();
        if (Hg) *Hg = Hnorm * Hlm_ * Hlm * Hng * Hg1 + Hnorm * Hlm_ * Hlm * Hog * Hg2;
        if (Hc) *Hc = - epsilon_ * Hz * Hnc * Hc1 - epsilon_ * Hz * Hoc * Hc2;
        if (Hv) *Hv = gtsam::Matrix16::Zero();
        return (gtsam::Vector1() << Energy - epsilon_ * z).finished();
      }
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 5;
  }

}; // \class MinForceFactor

} /// namespace gtsam_packing