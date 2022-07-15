#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

class StiffnessFactor3: public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector9> {

private:
  bool zj;

public:

  StiffnessFactor3(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, bool zeroJac) :
      gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector9>(
        gtsam::noiseModel::Isotropic::Sigma(6,1), key1, key2, key3, key4, key5), zj(zeroJac) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
    const gtsam::Pose3& p3, const gtsam::Pose3& p4, const gtsam::Vector9& v,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none,
    boost::optional<gtsam::Matrix&> H4 = boost::none,
    boost::optional<gtsam::Matrix&> H5 = boost::none) const {
      
      gtsam::Pose3 p3_1 = gtsam::traits<gtsam::Pose3>::Between(p1,p3,H1,H3);
      gtsam::Pose3 p4_2 = gtsam::traits<gtsam::Pose3>::Between(p2,p4,H2,H4);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H13;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H24;
      gtsam::Pose3 hx = gtsam::traits<gtsam::Pose3>::Between(p3_1,p4_2,&H13,&H24);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlm;
      gtsam::Vector6 lm = gtsam::traits<gtsam::Pose3>::Logmap(hx, &Hlm);

      gtsam::Vector lmc = (gtsam::Vector6() << lm[0], lm[1], lm[2], lm[3]+lm[1]*v[8]-lm[2]*v[7], lm[4]+lm[2]*v[6]-lm[0]*v[8], lm[5]+lm[0]*v[7]-lm[1]*v[6]).finished();
      gtsam::Matrix Hlmclm = (gtsam::Matrix66() << 1, 0, 0, 0, 0, 0,
                                                   0, 1, 0, 0, 0, 0,
                                                   0, 0, 1, 0, 0, 0,
                                                   0, v[8], -v[7], 1, 0, 0,
                                                   -v[8], 0, v[6], 0, 1, 0,
                                                   v[7], -v[6], 0, 0, 0, 1).finished();

      gtsam::Vector s = v;//(gtsam::Vector6() << v[0], pow(v[1],-0.5)*v[0], pow(v[2],-0.5)*v[0], pow(v[3],-0.5)*v[5], pow(v[4],-0.5)*v[5], v[5]).finished();
      /*
      gtsam::Matrix66 Hsv = (gtsam::Matrix66() << 1, 0, 0, 0, 0, 0,
                                                  pow(v[1],-0.5), -0.5*pow(v[1],-1.5)*v[0], 0, 0, 0, 0,
                                                  pow(v[2],-0.5), 0, -0.5*pow(v[2],-1.5)*v[0], 0, 0, 0,
                                                  0, 0, 0, 1, 0, 0,
                                                  0, 0, 0, pow(v[4],-0.5), -0.5*pow(v[4],-1.5)*v[3], 0,
                                                  0, 0, 0, pow(v[4],-0.5), 0, -0.5*pow(v[5],-1.5)*v[3]).finished();
      */
      gtsam::Vector6 lmc_ = gtsam::ediv_(lmc, s);
      gtsam::Matrix66 Hlmc_ = (gtsam::Matrix66() << 1/s[0], 0, 0, 0, 0, 0,
                                                   0, 1/s[1], 0, 0, 0, 0,
                                                   0, 0, 1/s[2], 0, 0, 0,
                                                   0, 0, 0, 1/s[3], 0, 0,
                                                   0, 0, 0, 0, 1/s[4], 0,
                                                   0, 0, 0, 0, 0, 1/s[5]).finished();

      if (zj==true) {
        if (H1) *H1 = gtsam::Matrix66::Zero();
        if (H3) *H3 = gtsam::Matrix66::Zero();
      } else {
        if (H1) *H1 = Hlmc_ * Hlmclm * Hlm * H13 * (*H1);
        if (H3) *H3 = Hlmc_ * Hlmclm * Hlm * H13 * (*H3);
      }        
      
      if (H2) *H2 = Hlmc_ * Hlmclm * Hlm * H24 * (*H2);
      if (H4) *H4 = Hlmc_ * Hlmclm * Hlm * H24 * (*H4);
      if (H5) *H5 = gtsam::Matrix69::Zero();

      return lmc_;
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 5;
  }

}; // \class StiffnessFactor3

} /// namespace gtsam_packing