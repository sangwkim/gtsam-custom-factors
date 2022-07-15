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
Vector9: Compliance + Acting Point Offset (x,y,z)
*/

class StiffnessRatioFactor3: public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector9> {

private:

  gtsam::Vector6 v_;

public:

  StiffnessRatioFactor3(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, gtsam::Key key5, gtsam::Key key6,
    const gtsam::Vector6 v_nominal, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3, gtsam::Vector9>(model, key1, key2, key3, key4, key5, key6),
      v_(v_nominal) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1, const gtsam::Pose3& p2,
    const gtsam::Pose3& p3, const gtsam::Pose3& p4, const gtsam::Pose3& p5, const gtsam::Vector9& v,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none,
    boost::optional<gtsam::Matrix&> H4 = boost::none,
    boost::optional<gtsam::Matrix&> H5 = boost::none,
    boost::optional<gtsam::Matrix&> H6 = boost::none) const {
      
      gtsam::Pose3 p13 = gtsam::traits<gtsam::Pose3>::Between(p1,p3,H1,H3);
      gtsam::Pose3 p24 = gtsam::traits<gtsam::Pose3>::Between(p2,p4,H2,H4);
      
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H13;
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H24;
      gtsam::Pose3 hx = gtsam::traits<gtsam::Pose3>::Between(p13,p24,&H13,&H24);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian H4_;
      gtsam::Pose3 p45 = gtsam::traits<gtsam::Pose3>::Between(p4,p5,&H4_,H5);
      gtsam::Point3 p45t = p45.translation();
      gtsam::Matrix Htr = (gtsam::Matrix36() << 0, 0, 0, 1, 0, 0,
                                  0, 0, 0, 0, 1, 0,
                                  0, 0, 0, 0, 0, 1).finished();

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlm;
      gtsam::Vector lm = gtsam::traits<gtsam::Pose3>::Logmap(hx,&Hlm);

      gtsam::Vector lmc = (gtsam::Vector6() << lm[0], lm[1], lm[2], lm[3]+lm[1]*v[8]-lm[2]*v[7], lm[4]+lm[2]*v[6]-lm[0]*v[8], lm[5]+lm[0]*v[7]-lm[1]*v[6]).finished();
      gtsam::Point3 p45tc = (gtsam::Vector3() << p45t.x()-v[6], p45t.y()-v[7], p45t.z()-v[8]).finished();

      gtsam::Matrix Hlmcv = (gtsam::Matrix69() << 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, -lm[2], lm[1],
                                                0, 0, 0, 0, 0, 0, lm[2], 0, -lm[0],
                                                0, 0, 0, 0, 0, 0, -lm[1], lm[0], 0).finished();

      gtsam::Matrix Hlmclm = (gtsam::Matrix66() << 1, 0, 0, 0, 0, 0,
                                                   0, 1, 0, 0, 0, 0,
                                                   0, 0, 1, 0, 0, 0,
                                                   0, v[8], -v[7], 1, 0, 0,
                                                   -v[8], 0, v[6], 0, 1, 0,
                                                   v[7], -v[6], 0, 0, 0, 1).finished();

      gtsam::Matrix Hcv = (gtsam::Matrix39() << 0, 0, 0, 0, 0, 0, -1, 0, 0,
                                                0, 0, 0, 0, 0, 0, 0, -1, 0,
                                                0, 0, 0, 0, 0, 0, 0, 0, -1).finished();

      //gtsam::Vector k = (gtsam::Vector6() << 1/v[0]/v[0], v[1]/v[0]/v[0], v[2]/v[0]/v[0], v[3]/v[5]/v[5], v[4]/v[5]/v[5], 1/v[5]/v[5]).finished();
      //gtsam::Vector k_ = (gtsam::Vector6() << 1/v_[0]/v_[0], v_[1]/v_[0]/v_[0], v_[2]/v_[0]/v_[0], v_[3]/v_[5]/v_[5], v_[4]/v_[5]/v_[5], 1/v_[5]/v_[5]).finished();
      gtsam::Vector k = (gtsam::Vector6() << 1/v[0]/v[0], 1/v[1]/v[1], 1/v[2]/v[2], 1/v[3]/v[3], 1/v[4]/v[4], 1/v[5]/v[5]).finished();
      gtsam::Vector k_ = (gtsam::Vector6() << 1/v_[0]/v_[0], 1/v_[1]/v_[1], 1/v_[2]/v_[2], 1/v_[3]/v_[3], 1/v_[4]/v_[4], 1/v_[5]/v_[5]).finished();

      /*
      gtsam::Matrix Hk = (gtsam::Matrix69() << -2/pow(v[0],3), 0, 0, 0, 0, 0, 0, 0, 0,
                                               -2*v[1]/pow(v[0],3), 1/pow(v[0],2), 0, 0, 0, 0, 0, 0, 0,
                                               -2*v[2]/pow(v[0],3), 0, 1/pow(v[0],2), 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 1/pow(v[5],2), 0, -2*v[3]/pow(v[5],3), 0, 0, 0,
                                               0, 0, 0, 0, 1/pow(v[5],2), -2*v[4]/pow(v[5],3), 0, 0, 0,
                                               0, 0, 0, 0, 0, -2/pow(v[5],3), 0, 0, 0).finished();
      */
      gtsam::Matrix Hk = (gtsam::Matrix69() << -2/pow(v[0],3), 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, -2/pow(v[1],3), 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, -2/pow(v[2],3), 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, -2/pow(v[3],3), 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, -2/pow(v[4],3), 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, -2/pow(v[5],3), 0, 0, 0).finished();

      gtsam::Matrix Hlmc = (gtsam::Matrix46() << k[0], 0, 0, 0, k[4]*p45tc.z(), -k[5]*p45tc.y(),
                                                 0, k[1], 0, -k[3]*p45tc.z(), 0, k[5]*p45tc.x(),
                                                 0, 0, k[2], k[3]*p45tc.y(), -k[4]*p45tc.x(), 0,
                                                 0, 0, 0, 0, 0, 0).finished();

      gtsam::Matrix Htr_ = (gtsam::Matrix43() << 0, -k[5]*lmc[5], k[4]*lmc[4],
                                                 k[5]*lmc[5], 0, -k[3]*lmc[3],
                                                 -k[4]*lmc[4], k[3]*lmc[3], 0,
                                                 0, 0, 0).finished();

      gtsam::Matrix Hkk = (gtsam::Matrix46() <<
                          lmc[0], 0, 0, 0, lmc[4]*p45tc.z(), -lmc[5]*p45tc.y(),
                          0, lmc[1], 0, -lmc[3]*p45tc.z(), 0, lmc[5]*p45tc.x(),
                          0, 0, lmc[2], lmc[3]*p45tc.y(), -lmc[4]*p45tc.x(), 0,
                          k[1]*k[2]*k[3]*k[4]*k[5]/(k_[0]*k_[1]*k_[2]*k_[3]*k_[4]*k_[5]), k[0]*k[2]*k[3]*k[4]*k[5]/(k_[0]*k_[1]*k_[2]*k_[3]*k_[4]*k_[5]), k[0]*k[1]*k[3]*k[4]*k[5]/(k_[0]*k_[1]*k_[2]*k_[3]*k_[4]*k_[5]), k[0]*k[1]*k[2]*k[4]*k[5]/(k_[0]*k_[1]*k_[2]*k_[3]*k_[4]*k_[5]), k[0]*k[1]*k[2]*k[3]*k[5]/(k_[0]*k_[1]*k_[2]*k_[3]*k_[4]*k_[5]), k[0]*k[1]*k[2]*k[3]*k[4]/(k_[0]*k_[1]*k_[2]*k_[3]*k_[4]*k_[5])
                          ).finished();

      if (H1) *H1 = Hlmc * Hlmclm * Hlm * H13 * (*H1);
      if (H2) *H2 = Hlmc * Hlmclm * Hlm * H24 * (*H2);
      if (H3) *H3 = Hlmc * Hlmclm * Hlm * H13 * (*H3);
      if (H4) *H4 = Hlmc * Hlmclm * Hlm * H24 * (*H4) + Htr_ * Htr * H4_;
      if (H5) *H5 = Htr_ * Htr * (*H5);
      if (H6) *H6 = Hkk * Hk + Hlmc * Hlmcv + Htr_ * Hcv;

      return (gtsam::Vector4() << 
        k[0]*lmc[0] - k[5]*lmc[5]*p45tc.y() + k[4]*lmc[4]*p45tc.z(),
        k[1]*lmc[1] - k[3]*lmc[3]*p45tc.z() + k[5]*lmc[5]*p45tc.x(),
        k[2]*lmc[2] - k[4]*lmc[4]*p45tc.x() + k[3]*lmc[3]*p45tc.y(),
        k[0]*k[1]*k[2]*k[3]*k[4]*k[5]/(k_[0]*k_[1]*k_[2]*k_[3]*k_[4]*k_[5]) - 1
        ).finished();
      
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 6;
  }

}; // \class StiffnessRatioFactor3

} /// namespace gtsam_packing