#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>
#include <cmath>

namespace gtsam_packing {


class InequalityFactor: public gtsam::NoiseModelFactor1<gtsam::Vector1> {

private:

  double b_;

public:

  InequalityFactor(gtsam::Key key1, double b, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor1<gtsam::Vector1>(model, key1),
        b_(b) {}

  gtsam::Vector evaluateError(
    const gtsam::Vector1& v,
    boost::optional<gtsam::Matrix&> H1 = boost::none) const {

      if (v[0] > 0) {
        if (H1) *H1 = gtsam::Matrix11::Zero();
        return (gtsam::Vector1() << 0.0).finished(); 
      } else {
        if (H1) *H1 = (gtsam::Matrix11() << 1).finished();
        return (gtsam::Vector1() << v[0] - b_).finished(); 
      }
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 1;
  }

}; // \class InequalityFactor

} /// namespace gtsam_packing