#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>
#include <cmath>

namespace gtsam_packing {

class CoulombFactor2: public gtsam::NoiseModelFactor1<gtsam::Pose3> {

/*
key1: O(i)

const gtsam::Pose3 O_0
const gtsam::Pose3 G_0
const gtsam::Pose3 G_i
const gtsam::Pose3 Q_i
const double E_N
const double L_N
const double MU
const gtsam::Vector6 S

*/

private:
  gtsam::Pose3 o_0;
  gtsam::Pose3 g_0;
  gtsam::Pose3 g_i;
  gtsam::Pose3 q_i;
  double e_n;
  double l_n;
  double mu;
  gtsam::Vector6 s;

public:

  CoulombFactor2(gtsam::Key key1,
    gtsam::Pose3 O_0,
    gtsam::Pose3 G_0,
    gtsam::Pose3 G_i,
    gtsam::Pose3 Q_i,
    double E_N,
    double L_N,
    double MU,
    gtsam::Vector6 S,
    gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key1),
      o_0(O_0), g_0(G_0), g_i(G_i), q_i(Q_i), e_n(E_N), l_n(L_N), s(S), mu(MU) {}

  gtsam::Vector evaluateError(const gtsam::Pose3& o_i,
    boost::optional<gtsam::Matrix&> H1 = boost::none) const {

      double eps = 1e-16;

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hdum;
      
      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hogi;
      gtsam::Pose3 og_0 = gtsam::traits<gtsam::Pose3>::Between(o_0,g_0);
      gtsam::Pose3 og_i = gtsam::traits<gtsam::Pose3>::Between(o_i,g_i,&Hogi,&Hdum);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hog0i;
      gtsam::Pose3 og_0i = gtsam::traits<gtsam::Pose3>::Between(og_0,og_i,&Hdum,&Hog0i);

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hlm;
      gtsam::Vector6 lm = gtsam::traits<gtsam::Pose3>::Logmap(og_0i, &Hlm);

      gtsam::Vector6 lm_ = gtsam::ediv_(lm, s);
      gtsam::Matrix66 Hlm_ = (gtsam::Matrix66() << 1/s[0], 0, 0, 0, 0, 0,
                                   0, 1/s[1], 0, 0, 0, 0,
                                   0, 0, 1/s[2], 0, 0, 0,
                                   0, 0, 0, 1/s[3], 0, 0,
                                   0, 0, 0, 0, 1/s[4], 0,
                                   0, 0, 0, 0, 0, 1/s[5]).finished();

      double e_t = lm_.squaredNorm();
      gtsam::Matrix16 Hsn = (gtsam::Matrix16() << -2*lm_[0], -2*lm_[1], -2*lm_[2], -2*lm_[3], -2*lm_[4], -2*lm_[5]).finished();

      typename gtsam::traits<gtsam::Pose3>::ChartJacobian::Jacobian Hqoi;
      gtsam::Pose3 qo_i = gtsam::traits<gtsam::Pose3>::Between(q_i,o_i,&Hdum,&Hqoi);
      
      gtsam::Point3 qo_i_t = qo_i.translation();
      gtsam::Matrix36 Ht = (gtsam::Matrix36() << 0, 0, 0, 1, 0, 0,
                                                 0, 0, 0, 0, 1, 0,
                                                 0, 0, 0, 0, 0, 1).finished();
      
      typename gtsam::Matrix13 Hn;
      double l_f = gtsam::norm3(qo_i_t, &Hn);

      double val = (std::abs(e_t) - e_n) * l_n - mu * (e_n * std::abs(l_f) + eps);

      if (val > 0 && e_t > 0) {
        if (H1) *H1 = l_n * Hsn * Hlm_ * Hlm * Hog0i * Hogi;
        return (gtsam::Vector1() << val).finished();
      } else if (val > 0 && e_t < 0) {
        if (H1) *H1 = - l_n * Hsn * Hlm_ * Hlm * Hog0i * Hogi;
        return (gtsam::Vector1() << val).finished();
      } else {
        if (H1) *H1 = gtsam::Matrix16::Zero();
        return (gtsam::Vector1() << 0.0).finished();
      }
      /*
      double val = (std::abs(e_t) - e_n) * l_n / (e_n * std::abs(l_f) + 1e-3);

      if (val > mu && e_t > 0 && l_f > 0) {
        if (H1) *H1 = l_n / (e_n * l_f + 1e-3) * Hsn * Hlm_ * Hlm * Hog0i * Hogi - (e_t - e_n) * l_n * e_n / pow(e_n * l_f + 1e-3, 2) * Hn * Ht * Hqoi;
        return (gtsam::Vector1() << val - mu).finished();
      } else if (val > mu && e_t > 0 && l_f < 0) {
        if (H1) *H1 = l_n / (e_n * l_f + 1e-3) * Hsn * Hlm_ * Hlm * Hog0i * Hogi + (e_t - e_n) * l_n * e_n / pow(e_n * l_f + 1e-3, 2) * Hn * Ht * Hqoi;
        return (gtsam::Vector1() << val - mu).finished();
      } else if (val > mu && e_t < 0 && l_f > 0) {
        if (H1) *H1 = - l_n / (e_n * l_f + 1e-3) * Hsn * Hlm_ * Hlm * Hog0i * Hogi - (e_t - e_n) * l_n * e_n / pow(e_n * l_f + 1e-3, 2) * Hn * Ht * Hqoi;
        return (gtsam::Vector1() << val - mu).finished();
      } else if (val > mu && e_t < 0 && l_f < 0) {
        if (H1) *H1 = - l_n / (e_n * l_f + 1e-3) * Hsn * Hlm_ * Hlm * Hog0i * Hogi + (e_t - e_n) * l_n * e_n / pow(e_n * l_f + 1e-3, 2) * Hn * Ht * Hqoi;
        return (gtsam::Vector1() << val - mu).finished();
      } else {
        if (H1) *H1 = gtsam::Matrix16::Zero();
        return (gtsam::Vector1() << 0.0).finished();
      */
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 1;
  }

}; // \class CoulombFactor2

} /// namespace gtsam_packing