#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>

namespace gtsam_packing {

class ConsistentDeformFactor: public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, 
                                                        gtsam::Pose3, gtsam::Pose3, gtsam::Pose3> {

public:

  ConsistentDeformFactor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, gtsam::Key key6,
    gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(model, key1, key2, key3, key4, key5, key6) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2, const gtsam::Pose3& p3,
    const gtsam::Pose3& p4, const gtsam::Pose3& p5, const gtsam::Pose3& p6,
    boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none, boost::optional<gtsam::Matrix&> H4 = boost::none,
    boost::optional<gtsam::Matrix&> H5 = boost::none, boost::optional<gtsam::Matrix&> H6 = boost::none) const {

      gtsam::Pose3 p14 = gtsam::traits<gtsam::Pose3>::Between(p1,p4,H1,H4);
      gtsam::Pose3 p25 = gtsam::traits<gtsam::Pose3>::Between(p2,p5,H2,H5);
      gtsam::Pose3 p36 = gtsam::traits<gtsam::Pose3>::Between(p3,p6,H3,H6);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H142;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H143;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H25;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H36;
      gtsam::Pose3 p1425 = gtsam::traits<gtsam::Pose3>::Between(p14,p25,&H142,&H25);
      gtsam::Pose3 p1436 = gtsam::traits<gtsam::Pose3>::Between(p14,p36,&H143,&H36);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H1425;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H1436;
      gtsam::Vector6 l1425 = gtsam::traits<gtsam::Pose3>::Logmap(p1425, &H1425);
      gtsam::Vector6 l1436 = gtsam::traits<gtsam::Pose3>::Logmap(p1436, &H1436);

      gtsam::Point3 l1425r = (gtsam::Point3() << l1425(0), l1425(1), l1425(2)).finished();
      gtsam::Point3 l1436r = (gtsam::Point3() << l1436(0), l1436(1), l1436(2)).finished();
      gtsam::Point3 l1425t = (gtsam::Point3() << l1425(3), l1425(4), l1425(5)).finished();
      gtsam::Point3 l1436t = (gtsam::Point3() << l1436(3), l1436(4), l1436(5)).finished();

      typename gtsam::Matrix13 H1425tnorm;
      typename gtsam::Matrix13 H1436tnorm;
      double l1425tnorm = gtsam::norm3(l1425t, &H1425tnorm);
      double l1436tnorm = gtsam::norm3(l1436t, &H1436tnorm);
      gtsam::Vector6 l1436_ = l1425tnorm / l1436tnorm * l1436;

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H1436_e;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H1425_;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H1436_;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H14251436l;
      gtsam::Pose3 p1436_ = gtsam::traits<gtsam::Pose3>::Expmap(l1436_, &H1436_e);
      gtsam::Pose3 p14251436_ = gtsam::traits<gtsam::Pose3>::Between(p1425, p1436_, &H1425_, &H1436_);
      gtsam::Vector6 l = gtsam::traits<gtsam::Pose3>::Logmap(p14251436_, &H14251436l);

      if (H1) *H1 = H14251436l * (H1425_ * H142 + H1436_ * H1436_e * l1425tnorm / l1436tnorm * H1436 * H143) * (*H1);
      if (H2) *H2 = H14251436l * H1425_ * H25 * (*H2);
      if (H3) *H3 = H14251436l * H1436_ * H1436_e * l1425tnorm / l1436tnorm * H1436 * H36 * (*H3);
      if (H4) *H4 = H14251436l * (H1425_ * H142 + H1436_ * H1436_e * l1425tnorm / l1436tnorm * H1436 * H143) * (*H4);
      if (H5) *H5 = H14251436l * H1425_ * H25 * (*H5);
      if (H6) *H6 = H14251436l * H1436_ * H1436_e * l1425tnorm / l1436tnorm * H1436 * H36 * (*H6);

      return l;

      /*
      gtsam::Point3 l1425r = (gtsam::Point3() << l1425(0), l1425(1), l1425(2)).finished();
      gtsam::Point3 l1436r = (gtsam::Point3() << l1436(0), l1436(1), l1436(2)).finished();
      gtsam::Point3 l1425t = (gtsam::Point3() << l1425(3), l1425(4), l1425(5)).finished();
      gtsam::Point3 l1436t = (gtsam::Point3() << l1436(3), l1436(4), l1436(5)).finished();

      typename gtsam::Matrix33 H1425r;
      typename gtsam::Matrix33 H1436r;
      typename gtsam::Matrix33 H1425t;
      typename gtsam::Matrix33 H1436t;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H1425rt;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H1436rt;
      gtsam::Point3 misalign_r = gtsam::cross(l1425r, l1436r, &H1425r, &H1436r);
      gtsam::Point3 misalign_t = gtsam::cross(l1425t, l1436t, &H1425t, &H1436t);

      H1425rt = (gtsam::Matrix66() << H1425r(0,0), H1425r(0,1), H1425r(0,2),          0.,          0.,          0.,
                                      H1425r(1,0), H1425r(1,1), H1425r(1,2),          0.,          0.,          0.,
                                      H1425r(2,0), H1425r(2,1), H1425r(2,2),          0.,          0.,          0.,
                                               0.,          0.,          0., H1425t(0,0), H1425t(0,1), H1425t(0,2),
                                               0.,          0.,          0., H1425t(1,0), H1425t(1,1), H1425t(1,2),
                                               0.,          0.,          0., H1425t(2,0), H1425t(2,1), H1425t(2,2)).finished();
      H1436rt = (gtsam::Matrix66() << H1436r(0,0), H1436r(0,1), H1436r(0,2),          0.,          0.,          0.,
                                      H1436r(1,0), H1436r(1,1), H1436r(1,2),          0.,          0.,          0.,
                                      H1436r(2,0), H1436r(2,1), H1436r(2,2),          0.,          0.,          0.,
                                               0.,          0.,          0., H1436t(0,0), H1436t(0,1), H1436t(0,2),
                                               0.,          0.,          0., H1436t(1,0), H1436t(1,1), H1436t(1,2),
                                               0.,          0.,          0., H1436t(2,0), H1436t(2,1), H1436t(2,2)).finished();

      if (H1) *H1 = H1425rt * H1425 * H142 * (*H1) + H1436rt * H1436 * H143 * (*H1);
      if (H2) *H2 = H1425rt * H1425 * H25 * (*H2);
      if (H3) *H3 = H1436rt * H1436 * H36 * (*H3);
      if (H4) *H4 = H1425rt * H1425 * H142 * (*H4) + H1436rt * H1436 * H143 * (*H4);
      if (H5) *H5 = H1425rt * H1425 * H25 * (*H5);
      if (H6) *H6 = H1436rt * H1436 * H36 * (*H6);

      return (gtsam::Vector6() << misalign_r(0), misalign_r(1), misalign_r(2),
                                  misalign_t(0), misalign_t(1), misalign_t(2)).finished();
      */

  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 6;
  }

};

} /// namespace gtsam_packing