#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

/*
Pose3: Q(i)
Pose3: O(i)
Vector1: D(i) [EnergyWOF, EnergyWF, LenSlip, LenSlip]
*/

class LenSlipFactor: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Vector1> {

public:

  LenSlipFactor(gtsam::Key ks1, gtsam::Key ks2, gtsam::Key ks3, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Vector1>(
        //gtsam::noiseModel::Constrained::All(1), ks1, ks2, ks3) {}
        model, ks1, ks2, ks3) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
    const gtsam::Vector1& v3,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none) const {
      
      gtsam::Pose3 p12 = gtsam::traits<gtsam::Pose3>::Between(p1, p2, H1, H2);

      //typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlm;
      //gtsam::Vector6 lm = gtsam::traits<gtsam::Pose3>::Logmap(p12, &Hlm);

      gtsam::Point3 pt12 = p12.translation();
      pt12[2] = 0;
      gtsam::Matrix36 Ht = (gtsam::Matrix36() << 0, 0, 0, 1, 0, 0,
                                                 0, 0, 0, 0, 1, 0,
                                                 0, 0, 0, 0, 0, 0).finished();

      typename gtsam::Matrix13 Hnorm;
      double slip = gtsam::norm3(pt12, &Hnorm);

      if (H1) *H1 = gtsam::Matrix16::Zero();// - Hnorm * Ht * (*H1);
      if (H2) *H2 = - Hnorm * Ht * (*H2);
      if (H3) *H3 = (gtsam::Matrix11() << 1).finished();

      return (gtsam::Vector1() << v3[0] - slip).finished();
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 3;
  }

}; // \class LenSlipFactor

} /// namespace gtsam_packing