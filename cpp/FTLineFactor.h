#pragma once

#include <ostream>
#include <cmath>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

/*
Pose3: N(i)
Pose3: O(i)
Pose3: G(i)
Pose3: C(i)
Vector6: Compliance
*/

class FTLineFactor: public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6> {

private:

  bool zj;

public:

  FTLineFactor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5,
    gtsam::SharedNoiseModel model, bool zeroJac) :
      gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6>(model, key1, key2, key3, key4, key5), zj(zeroJac) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pn, const gtsam::Pose3& po,
    const gtsam::Pose3& pg, const gtsam::Pose3& pc, const gtsam::Vector6& v,
    boost::optional<gtsam::Matrix&> Hn = boost::none,
    boost::optional<gtsam::Matrix&> Ho = boost::none,
    boost::optional<gtsam::Matrix&> Hg = boost::none,
    boost::optional<gtsam::Matrix&> Hc = boost::none,
    boost::optional<gtsam::Matrix&> Hv = boost::none) const {

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hg_, Hlmng, Hlmog, Hg__;
      typename gtsam::traits<gtsam::Rot3>::ChartJacobian::Jacobian Haro;
      typename gtsam::traits<gtsam::Point3>::ChartJacobian::Jacobian Haft;
      /////////////
      gtsam::Pose3 png = gtsam::traits<gtsam::Pose3>::Between(pn,pg,Hn,Hg);
      gtsam::Pose3 pog = gtsam::traits<gtsam::Pose3>::Between(po,pg,Ho,&Hg_);
      ///////////// 
      gtsam::Vector lm = gtsam::traits<gtsam::Pose3>::Local(png,pog,&Hlmng,&Hlmog);
      /////////////
      gtsam::Pose3 pgc = gtsam::traits<gtsam::Pose3>::Between(pg,pc,&Hg__,Hc);
      gtsam::Point3 pgct = pgc.translation();
      gtsam::Matrix Htr = (gtsam::Matrix36() << 0, 0, 0, 1, 0, 0,
                                                0, 0, 0, 0, 1, 0,
                                                0, 0, 0, 0, 0, 1).finished();
      /////////////
      gtsam::Vector k = (gtsam::Vector6() << 1/v[0]/v[0], 1/v[1]/v[1], 1/v[2]/v[2], 1/v[3]/v[3], 1/v[4]/v[4], 1/v[5]/v[5]).finished();
      gtsam::Matrix Hk = (gtsam::Matrix66() << -2/pow(v[0],3), 0, 0, 0, 0, 0,
                                                0, -2/pow(v[1],3), 0, 0, 0, 0,
                                                0, 0, -2/pow(v[2],3), 0, 0, 0,
                                                0, 0, 0, -2/pow(v[3],3), 0, 0,
                                                0, 0, 0, 0, -2/pow(v[4],3), 0,
                                                0, 0, 0, 0, 0, -2/pow(v[5],3)).finished();
      /////////////
      gtsam::Vector ft = (gtsam::Vector3() << k[0]*lm[0] - k[5]*lm[5]*pgct.y() + k[4]*lm[4]*pgct.z(),
                                              k[1]*lm[1] - k[3]*lm[3]*pgct.z() + k[5]*lm[5]*pgct.x(),
                                              k[2]*lm[2] - k[4]*lm[4]*pgct.x() + k[3]*lm[3]*pgct.y()).finished();
      gtsam::Matrix Hftlm = (gtsam::Matrix36() << k[0], 0, 0, 0, k[4]*pgct.z(), -k[5]*pgct.y(),
                                                 0, k[1], 0, -k[3]*pgct.z(), 0, k[5]*pgct.x(),
                                                 0, 0, k[2], k[3]*pgct.y(), -k[4]*pgct.x(), 0).finished();
      gtsam::Matrix Hfttr = (gtsam::Matrix33() << 0, -k[5]*lm[5], k[4]*lm[4],
                                                 k[5]*lm[5], 0, -k[3]*lm[3],
                                                 -k[4]*lm[4], k[3]*lm[3], 0).finished();
      gtsam::Matrix Hftk = (gtsam::Matrix36() << lm[0], 0, 0, 0, lm[4]*pgct.z(), -lm[5]*pgct.y(),
                                                  0, lm[1], 0, -lm[3]*pgct.z(), 0, lm[5]*pgct.x(),
                                                  0, 0, lm[2], lm[3]*pgct.y(), -lm[4]*pgct.x(), 0).finished();
      /////////////
      double fta = pgc.rotation().unrotate(ft, &Haro, &Haft).x();
      gtsam::Matrix Hro = (gtsam::Matrix36() << 1, 0, 0, 0, 0, 0,
                                                0, 1, 0, 0, 0, 0,
                                                0, 0, 1, 0, 0, 0).finished();
      gtsam::Matrix Hx = (gtsam::Matrix13() << 1, 0, 0).finished();
      /////////////
      if (zj==true) {
        if (Hn) *Hn = gtsam::Matrix16::Zero();
        if (Ho) *Ho = gtsam::Matrix16::Zero();
        if (Hg) *Hg = gtsam::Matrix16::Zero();
        if (Hv) *Hv = gtsam::Matrix16::Zero();
      } else {
        if (Hn) *Hn = Hx * Haft * Hftlm * Hlmng * (*Hn);
        if (Ho) *Ho = Hx * Haft * Hftlm * Hlmog * (*Ho);
        if (Hg) *Hg = Hx * (Haro * Hro * Hg__ + Haft * (Hftlm * (Hlmng * (*Hg) + Hlmog * Hg_) + Hfttr * Htr * Hg__));
        if (Hv) *Hv = Hx * Haft * Hftk * Hk;
      }
      if (Hc) *Hc = Hx * (Haro * Hro * (*Hc) + Haft * Hfttr * Htr * (*Hc));

      return (gtsam::Vector1() << fta).finished();
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 6;
  }

}; //

} /// namespace gtsam_packing
