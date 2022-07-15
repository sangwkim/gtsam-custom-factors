#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

class TactileTransformFactor_3D_nonlinear: public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3> {

private:

  gtsam::Vector6 k_;
  gtsam::Vector6 alpha_;

public:

  TactileTransformFactor_3D_nonlinear(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4,
    const gtsam::Vector6 k, const gtsam::Vector6 alpha) :
      gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(gtsam::noiseModel::Isotropic::Sigma(6,1), key1, key2, key3, key4),
      k_(k), alpha_(alpha) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
    const gtsam::Pose3& p3, const gtsam::Pose3& p4, boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none, boost::optional<gtsam::Matrix&> H3 = boost::none,
    boost::optional<gtsam::Matrix&> H4 = boost::none) const {

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H13, H24;
      gtsam::Pose3 p13 = gtsam::traits<gtsam::Pose3>::Between(p1,p3,H1,H3);
      gtsam::Pose3 p24 = gtsam::traits<gtsam::Pose3>::Between(p2,p4,H2,H4);
      gtsam::Vector lm = gtsam::traits<gtsam::Pose3>::Local(p13,p24,&H13,&H24);
      gtsam::Vector err = (gtsam::Vector6() << pow(k_[0],.5)*lm[0] - .25*alpha_[0]*pow(k_[0],-.5)*pow(lm[0],3),
                                               pow(k_[1],.5)*lm[1] - .25*alpha_[1]*pow(k_[1],-.5)*pow(lm[1],3),
                                               pow(k_[2],.5)*lm[2] - .25*alpha_[2]*pow(k_[2],-.5)*pow(lm[2],3),
                                               pow(k_[3],.5)*lm[3] - .25*alpha_[3]*pow(k_[3],-.5)*pow(lm[3],3),
                                               pow(k_[4],.5)*lm[4] - .25*alpha_[4]*pow(k_[4],-.5)*pow(lm[4],3),
                                               pow(k_[5],.5)*lm[5] - .25*alpha_[5]*pow(k_[5],-.5)*pow(lm[5],3)).finished();
      gtsam::Matrix66 Herr = (gtsam::Matrix66() << pow(k_[0],.5) - .75*alpha_[0]*pow(k_[0],-.5)*pow(lm[0],2), 0, 0, 0, 0, 0,
                                                   0, pow(k_[1],.5) - .75*alpha_[1]*pow(k_[1],-.5)*pow(lm[1],2), 0, 0, 0, 0,
                                                   0, 0, pow(k_[2],.5) - .75*alpha_[2]*pow(k_[2],-.5)*pow(lm[2],2), 0, 0, 0,
                                                   0, 0, 0, pow(k_[3],.5) - .75*alpha_[3]*pow(k_[3],-.5)*pow(lm[3],2), 0, 0,
                                                   0, 0, 0, 0, pow(k_[4],.5) - .75*alpha_[4]*pow(k_[4],-.5)*pow(lm[4],2), 0,
                                                   0, 0, 0, 0, 0, pow(k_[5],.5) - .75*alpha_[5]*pow(k_[5],-.5)*pow(lm[5],2)).finished();

      if (H1) *H1 = Herr * H13 * (*H1);
      if (H2) *H2 = Herr * H24 * (*H2);
      if (H3) *H3 = Herr * H13 * (*H3);
      if (H4) *H4 = Herr * H24 * (*H4);

      return err;
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 4;
  }

}; // \class TactileTransformFactor_3D_nonlinear

} /// namespace gtsam_packing