#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

class PenetrationFactor: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {

private:

  double epsilon_;

public:

  PenetrationFactor(gtsam::Key key1, gtsam::Key key2, double cost_sigma, double eps) :
      gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), key1, key2),
      epsilon_(eps) {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3& p1,
    const gtsam::Pose3& p2,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none
    //boost::optional<gtsam::Matrix&> Hz = boost::none
    ) const {

      gtsam::Pose3 p12 = gtsam::traits<gtsam::Pose3>::Between(p1, p2, H1, H2);
      double p12z = p12.translation().z();

      double val;
      //gtsam::OptionalJacobian<1,6> Hz;
      gtsam::Matrix16 Hz;

      if (p12z > epsilon_) {
        //if (Hz) *
        Hz = gtsam::Matrix16::Zero();
        val = 0.0;
      } else {
        //if (Hz) *
        Hz = -(gtsam::Matrix16() << 0, 0, 0, 0, 0, 1).finished();
        val = epsilon_ - p12z;
      }

      if (H1) *H1 = Hz * (*H1);
      if (H2) *H2 = Hz * (*H2);

      return (gtsam::Vector1() << val).finished();
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 2;
  }

}; // \class PenetrationFactor

} /// namespace gtsam_packing