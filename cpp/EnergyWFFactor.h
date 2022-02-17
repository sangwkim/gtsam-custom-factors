#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>
#include <cmath>

namespace gtsam_packing {

class EnergyWFFactor: public gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector1> {

private:

  gtsam::Vector6 s_;

public:

  EnergyWFFactor(gtsam::Key kwf1, gtsam::Key kwf2, gtsam::Key kwf3, gtsam::Key kwf4, gtsam::Key kwf5,
    const gtsam::Vector6 swf, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor5<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector1>(
        //gtsam::noiseModel::Constrained::All(1), kwf1, kwf2, kwf3, kwf4, kwf5),
        model, kwf1, kwf2, kwf3, kwf4, kwf5),
        s_(swf) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
    const gtsam::Pose3& p3, const gtsam::Pose3& p4, const gtsam::Vector1& v5,
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

      gtsam::Vector6 lm_ = gtsam::ediv_(lm, s_);
      gtsam::Matrix66 Hlm_ = (gtsam::Matrix66() << 1/s_[0], 0, 0, 0, 0, 0,
                                   0, 1/s_[1], 0, 0, 0, 0,
                                   0, 0, 1/s_[2], 0, 0, 0,
                                   0, 0, 0, 1/s_[3], 0, 0,
                                   0, 0, 0, 0, 1/s_[4], 0,
                                   0, 0, 0, 0, 0, 1/s_[5]).finished();

      gtsam::Matrix16 Hnorm = (gtsam::Matrix16() << -2*lm_[0], -2*lm_[1], -2*lm_[2], -2*lm_[3], -2*lm_[4], -2*lm_[5]).finished();
      
      if (H1) *H1 = Hnorm * Hlm_ * Hlm * H13 * (*H1);
      if (H2) *H2 = Hnorm * Hlm_ * Hlm * H24 * (*H2);
      if (H3) *H3 = Hnorm * Hlm_ * Hlm * H13 * (*H3);
      if (H4) *H4 = Hnorm * Hlm_ * Hlm * H24 * (*H4);
      if (v5[0] > 0) {
        if (H5) *H5 = (gtsam::Matrix11() << 1).finished();
      } else {
        if (H5) *H5 = (gtsam::Matrix11() << -1).finished();
      }
      
      return (gtsam::Vector1() << std::abs(v5[0]) - lm_.squaredNorm()).finished();
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 5;
  }

}; // \class EnergyWFFactor

} /// namespace gtsam_packing