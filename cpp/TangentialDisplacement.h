#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

// n, c, o
class TangentialDisplacement: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3> {

public:

  TangentialDisplacement(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, double cost_sigma) :
      gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(gtsam::noiseModel::Isotropic::Sigma(1, cost_sigma), key1, key2, key3) {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3& n,
    const gtsam::Pose3& c,
    const gtsam::Pose3& o,
    boost::optional<gtsam::Matrix&> Hn = boost::none,
    boost::optional<gtsam::Matrix&> Hc = boost::none,
    boost::optional<gtsam::Matrix&> Ho = boost::none
    ) const {

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Ho1, Hc1, Hn2, Hc2, Hoc, Hnc;
      gtsam::Matrix36 Hncoc;

      gtsam::Pose3 oc = gtsam::traits<gtsam::Pose3>::Between(o, c, &Ho1, &Hc1);
      gtsam::Pose3 nc = gtsam::traits<gtsam::Pose3>::Between(n, c, &Hn2, &Hc2);
      gtsam::Pose3 ncoc = gtsam::traits<gtsam::Pose3>::Between(nc, oc, &Hnc, &Hoc);
      gtsam::Point3 ncoct = ncoc.translation(&Hncoc);

      double d_squared = pow(ncoct.x(),2) + pow(ncoct.y(),2);
      gtsam::Matrix Hncoct = (gtsam::Matrix13() << 2*ncoct.x(), 2*ncoct.y(), 0).finished();
      double d = pow(d_squared, 0.5);
      double Hds = 0.5 * pow(d_squared, -0.5);

      if (Hn) *Hn = Hds*Hncoct*Hncoc*Hnc*Hn2;
      //if (Hc) *Hc = gtsam::Matrix16::Zero();
      //if (Ho) *Ho = gtsam::Matrix16::Zero();
      if (Hc) *Hc = Hds*Hncoct*Hncoc*(Hoc*Hc1+Hnc*Hc2);
      if (Ho) *Ho = Hds*Hncoct*Hncoc*Hoc*Ho1;

      return (gtsam::Vector1() << d).finished();
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 3;
  }

}; // \class TangentialDisplacement

} /// namespace gtsam_packing