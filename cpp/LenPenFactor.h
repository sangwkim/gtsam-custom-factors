#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

/*
Pose3: N(i)
Pose3: C(i)
Vector1: D(i) [EnergyWOF, EnergyWF, LenPen, LenSlip]
*/

class LenPenFactor: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Vector1> {

public:

  LenPenFactor(gtsam::Key kp1, gtsam::Key kp2, gtsam::Key kp3, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Vector1>(
        //gtsam::noiseModel::Constrained::All(1), kp1, kp2, kp3) {}
        model, kp1, kp2, kp3) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
    const gtsam::Vector1& v3,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none) const {
      
      gtsam::Pose3 p12 = gtsam::traits<gtsam::Pose3>::Between(p1, p2, H1, H2);
      
      //typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlm;
      //gtsam::Vector6 lm = gtsam::traits<gtsam::Pose3>::Logmap(p12, &Hlm);
      
      double p12z = p12.translation().z();//lm[5]; //
      gtsam::Matrix16 Hz = (gtsam::Matrix16() << 0, 0, 0, 0, 0, -1).finished();

      if (H1) *H1 = Hz * (*H1);
      if (H2) *H2 = Hz * (*H2);
      if (H3) *H3 = (gtsam::Matrix11() << 1).finished();
      
      return (gtsam::Vector1() << v3[0] - p12z).finished();
      
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 3;
  }

}; // \class LenPenFactor

} /// namespace gtsam_packing