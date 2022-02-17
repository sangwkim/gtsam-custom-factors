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

class LenPenFactor_2: public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector1> {

public:

  LenPenFactor_2(gtsam::Key kp1, gtsam::Key kp2, gtsam::Key kp3, gtsam::Key kp4, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector1>(
        model, kp1, kp2, kp3, kp4) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& pn, const gtsam::Pose3& pc, const gtsam::Pose3& po,
    const gtsam::Vector1& vx,
    boost::optional<gtsam::Matrix&> Hn = boost::none,
    boost::optional<gtsam::Matrix&> Hc = boost::none,
    boost::optional<gtsam::Matrix&> Ho = boost::none,
    boost::optional<gtsam::Matrix&> Hx = boost::none) const {

      gtsam::Pose3 poc = gtsam::traits<gtsam::Pose3>::Between(po, pc, Ho, Hc);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hoc;
      gtsam::Pose3 pnoc = gtsam::traits<gtsam::Pose3>::Compose(pn, poc, Hn, &Hoc);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hc_;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hnoc;
      gtsam::Pose3 pcnoc = gtsam::traits<gtsam::Pose3>::Between(pc, pnoc, &Hc_, &Hnoc);

      double pcnocz = - pcnoc.translation().z();
      gtsam::Matrix16 Hz = (gtsam::Matrix16() << 0, 0, 0, 0, 0, -1).finished();

      if (Hn) *Hn = - Hz * Hnoc * (*Hn);
      if (Hc) *Hc = - (Hz * Hnoc * Hoc * (*Hc) + Hz * Hc_);
      if (Ho) *Ho = - Hz * Hnoc * Hoc * (*Ho);
      if (Hx) *Hx = (gtsam::Matrix11() << 1).finished();
      
      return (gtsam::Vector1() << vx[0] - pcnocz).finished();
      
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 4;
  }

}; // \class LenPenFactor_2

} /// namespace gtsam_packing