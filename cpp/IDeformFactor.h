#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

/*
Computes
(G^-1 * N * O^-1 * G)_prev <--> (G^-1 * N * O^-1 * G)_next
*/
namespace gtsam_packing {

class IDeformFactor: public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3> {

public:

  IDeformFactor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, gtsam::Key key6,
    gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(model, key1, key2, key3, key4, key5, key6) {}

  gtsam::Vector evaluateError(
    const gtsam::Pose3& pn0, const gtsam::Pose3& po0, const gtsam::Pose3& pg0,
    const gtsam::Pose3& pn1, const gtsam::Pose3& po1, const gtsam::Pose3& pg1,
    boost::optional<gtsam::Matrix&> Hn0 = boost::none,
    boost::optional<gtsam::Matrix&> Ho0 = boost::none,
    boost::optional<gtsam::Matrix&> Hg0 = boost::none,
    boost::optional<gtsam::Matrix&> Hn1 = boost::none,
    boost::optional<gtsam::Matrix&> Ho1 = boost::none,
    boost::optional<gtsam::Matrix&> Hg1 = boost::none) const {
      
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hog0, Hg0_, Hnog0, Hgnog0, Hog1, Hg1_, Hnog1, Hgnog1;
      gtsam::Pose3 pog0 = gtsam::traits<gtsam::Pose3>::Between(po0,pg0,Ho0,Hg0);
      gtsam::Pose3 pnog0 = gtsam::traits<gtsam::Pose3>::Compose(pn0,pog0,Hn0,&Hog0);
      gtsam::Pose3 gnog0 = gtsam::traits<gtsam::Pose3>::Between(pg0,pnog0,&Hg0_,&Hnog0);
      gtsam::Pose3 pog1 = gtsam::traits<gtsam::Pose3>::Between(po1,pg1,Ho1,Hg1);
      gtsam::Pose3 pnog1 = gtsam::traits<gtsam::Pose3>::Compose(pn1,pog1,Hn1,&Hog1);
      gtsam::Pose3 gnog1 = gtsam::traits<gtsam::Pose3>::Between(pg1,pnog1,&Hg1_,&Hnog1);
      gtsam::Vector hx = gtsam::traits<gtsam::Pose3>::Local(gnog0,gnog1,&Hgnog0,&Hgnog1);

      if (Hn0) *Hn0 = Hgnog0 * Hnog0 * (*Hn0);
      if (Ho0) *Ho0 = Hgnog0 * Hnog0 * Hog0 * (*Ho0);
      if (Hg0) *Hg0 = Hgnog0 * Hg0_ + Hnog0 * Hog0 * (*Hg0);
      if (Hn1) *Hn1 = Hgnog1 * Hnog1 * (*Hn1);
      if (Ho1) *Ho1 = Hgnog1 * Hnog1 * Hog1 * (*Ho1);
      if (Hg1) *Hg1 = Hgnog1 * Hg1_ + Hnog1 * Hog1 * (*Hg1);

      return hx;
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 6;
  }

}; // \class IDeformFactor

} /// namespace gtsam_packing