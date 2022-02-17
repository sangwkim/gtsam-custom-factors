#pragma once

#include <ostream>
#include <cmath>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {


/*
Pose3: O(0)
Pose3: O(i)
Pose3: G(0)
Pose3: G(i)
Pose3: C(i)
Vector6: Compliance
*/

class StiffnessRatioFactor: public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6> {

private:

  gtsam::Vector6 v_;

public:

  StiffnessRatioFactor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, gtsam::Key key6,
    const gtsam::Vector6 v_nominal, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector6>(model, key1, key2, key3, key4, key5, key6),
      v_(v_nominal) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
    const gtsam::Pose3& p3, const gtsam::Pose3& p4, const gtsam::Pose3& p5, const gtsam::Vector6& v,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none,
    boost::optional<gtsam::Matrix&> H4 = boost::none,
    boost::optional<gtsam::Matrix&> H5 = boost::none,
    boost::optional<gtsam::Matrix&> H6 = boost::none) const {
      
      gtsam::Pose3 p13 = gtsam::traits<gtsam::Pose3>::Between(p1,p3,H1,H3);
      gtsam::Pose3 p24 = gtsam::traits<gtsam::Pose3>::Between(p2,p4,H2,H4);
      
      gtsam::Pose3 hx = gtsam::traits<gtsam::Pose3>::Between(p13,p24);

      gtsam::Pose3 p45 = gtsam::traits<gtsam::Pose3>::Between(p4,p5);
      gtsam::Point3 p45t = p45.translation();

      gtsam::Vector lm = gtsam::traits<gtsam::Pose3>::Logmap(hx);
      
      gtsam::Vector k = (gtsam::Vector6() << 1/v[0]/v[0], 1/v[1]/v[1], 1/v[2]/v[2], 1/v[3]/v[3], 1/v[4]/v[4], 1/v[5]/v[5]).finished();
      gtsam::Vector k_ = (gtsam::Vector6() << 1/v_[0]/v_[0], 1/v_[1]/v_[1], 1/v_[2]/v_[2], 1/v_[3]/v_[3], 1/v_[4]/v_[4], 1/v_[5]/v_[5]).finished();

      gtsam::Matrix Hk = (gtsam::Matrix66() << -2/pow(v[0],3), 0, 0, 0, 0, 0,
        0, -2/pow(v[1],3), 0, 0, 0, 0,
        0, 0, -2/pow(v[2],3), 0, 0, 0,
        0, 0, 0, -2/pow(v[3],3), 0, 0,
        0, 0, 0, 0, -2/pow(v[4],3), 0,
        0, 0, 0, 0, 0, -2/pow(v[5],3)).finished();

      *H1 = gtsam::Matrix46::Zero();
      *H2 = gtsam::Matrix46::Zero();
      *H3 = gtsam::Matrix46::Zero();
      *H4 = gtsam::Matrix46::Zero();
      *H5 = gtsam::Matrix46::Zero();
      *H6 = (gtsam::Matrix46() <<
        -lm[0], 0, 0, 0, lm[4]*p45t.z(), -lm[5]*p45t.y(),
        0, -lm[1], 0, -lm[3]*p45t.z(), 0, lm[5]*p45t.x(),
        0, 0, -lm[2], lm[3]*p45t.y(), -lm[4]*p45t.x(), 0,
        k[1]*k[2]*k[3]*k[4]*k[5], k[0]*k[2]*k[3]*k[4]*k[5], k[0]*k[1]*k[3]*k[4]*k[5], k[0]*k[1]*k[2]*k[4]*k[5], k[0]*k[1]*k[2]*k[3]*k[5], k[0]*k[1]*k[2]*k[3]*k[4]
        ).finished() * Hk;

      return (gtsam::Vector4() << 
        - k[0]*lm[0] - k[5]*lm[5]*p45t.y() + k[4]*lm[4]*p45t.z(),
        - k[1]*lm[1] - k[3]*lm[3]*p45t.z() + k[5]*lm[5]*p45t.x(),
        - k[2]*lm[2] - k[4]*lm[4]*p45t.x() + k[3]*lm[3]*p45t.y(),
        k[0]*k[1]*k[2]*k[3]*k[4]*k[5] - k_[0]*k_[1]*k_[2]*k_[3]*k_[4]*k_[5]
        ).finished();
      
      /*
      *H1 = gtsam::Matrix46::Zero();
      *H2 = gtsam::Matrix46::Zero();
      *H3 = gtsam::Matrix46::Zero();
      *H4 = gtsam::Matrix46::Zero();
      *H5 = gtsam::Matrix46::Zero();
      *H6 = (gtsam::Matrix46() <<
        -lm[0], 0, 0, 0, lm[4]*p45t.z(), -lm[5]*p45t.y(),
        0, -lm[1], 0, -lm[3]*p45t.z(), 0, lm[5]*p45t.x(),
        0, 0, -lm[2], lm[3]*p45t.y(), -lm[4]*p45t.x(), 0,
        v[1]*v[2]*v[3]*v[4]*v[5], v[0]*v[2]*v[3]*v[4]*v[5], v[0]*v[1]*v[3]*v[4]*v[5], v[0]*v[1]*v[2]*v[4]*v[5], v[0]*v[1]*v[2]*v[3]*v[5], v[0]*v[1]*v[2]*v[3]*v[4]
        ).finished();

      return (gtsam::Vector4() << 
        - v[0]*lm[0] - v[5]*lm[5]*p45t.y() + v[4]*lm[4]*p45t.z(),
        - v[1]*lm[1] - v[3]*lm[3]*p45t.z() + v[5]*lm[5]*p45t.x(),
        - v[2]*lm[2] - v[4]*lm[4]*p45t.x() + v[3]*lm[3]*p45t.y(),
        v[0]*v[1]*v[2]*v[3]*v[4]*v[5] - v_[0]*v_[1]*v_[2]*v_[3]*v_[4]*v_[5]
        ).finished();
      */      

  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 6;
  }

}; // \class StiffnessRatioFactor

} /// namespace gtsam_packing
