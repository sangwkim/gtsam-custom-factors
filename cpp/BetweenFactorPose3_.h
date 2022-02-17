#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

class BetweenFactorPose3_: public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {

private:

  gtsam::Pose3 p_;

public:

  BetweenFactorPose3_(gtsam::Key key1, gtsam::Key key2, gtsam::Pose3 p, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1, key2), p_(p) {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3& p1,
    const gtsam::Pose3& p2,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none
    ) const {

      gtsam::Pose3 p12 = gtsam::traits<gtsam::Pose3>::Between(p1, p2, H1, H2);
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlocal;
      gtsam::Vector rval = gtsam::traits<gtsam::Pose3>::Local(p_, p12, boost::none, &Hlocal);

      //std::cout << "rval: " << rval << std::endl << "p_: " << p_ << std::endl << "p12: " << p12 << std::endl;

      if (H1) *H1 = gtsam::Matrix66::Zero();
      if (H2) *H2 = Hlocal * (*H2);

      return rval;
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 2;
  }

}; // \class BetweenFactorPose3_

} /// namespace gtsam_packing