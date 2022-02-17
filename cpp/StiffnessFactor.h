#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

class StiffnessFactor: public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6> {

private:
  bool zj;

public:

  StiffnessFactor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, bool zeroJac) :
      gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6>(
        gtsam::noiseModel::Isotropic::Sigma(6,1), key1, key2, key3, key4, key5), zj(zeroJac) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
    const gtsam::Pose3& p3, const gtsam::Pose3& p4, const gtsam::Vector6& v,
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

      gtsam::Vector6 lm_ = gtsam::ediv_(lm, v);
      gtsam::Matrix66 Hlm_ = (gtsam::Matrix66() << 1/v[0], 0, 0, 0, 0, 0,
                                                   0, 1/v[1], 0, 0, 0, 0,
                                                   0, 0, 1/v[2], 0, 0, 0,
                                                   0, 0, 0, 1/v[3], 0, 0,
                                                   0, 0, 0, 0, 1/v[4], 0,
                                                   0, 0, 0, 0, 0, 1/v[5]).finished();

      if (zj==true) {
        if (H1) *H1 = gtsam::Matrix66::Zero();
        if (H3) *H3 = gtsam::Matrix66::Zero();
      } else {
        if (H1) *H1 = Hlm_ * Hlm * H13 * (*H1);
        if (H3) *H3 = Hlm_ * Hlm * H13 * (*H3);
      }        
      
      if (H2) *H2 = Hlm_ * Hlm * H24 * (*H2);      
      if (H4) *H4 = Hlm_ * Hlm * H24 * (*H4);
      if (H5) *H5 = gtsam::Matrix66::Zero();

      return lm_;
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 5;
  }

}; // \class StiffnessFactor

} /// namespace gtsam_packing