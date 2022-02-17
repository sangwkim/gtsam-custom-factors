#pragma once

#include <ostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/base/Lie.h>
#include <cmath>

namespace gtsam_packing {

/*
Vector4: D(i)
*/

class CoulombFactor: public gtsam::NoiseModelFactor4<gtsam::Vector1, gtsam::Vector1, gtsam::Vector1, gtsam::Vector1> {

private:

  double mu_;

public:

  CoulombFactor(gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, gtsam::Key key4, double mu, gtsam::SharedNoiseModel model) :
      gtsam::NoiseModelFactor4<gtsam::Vector1, gtsam::Vector1, gtsam::Vector1, gtsam::Vector1>(model, key1, key2, key3, key4),
        mu_(mu) {}

  gtsam::Vector evaluateError(
    const gtsam::Vector1& v1, const gtsam::Vector1& v2, const gtsam::Vector1& v3, const gtsam::Vector1& v4,
    boost::optional<gtsam::Matrix&> H1 = boost::none,
    boost::optional<gtsam::Matrix&> H2 = boost::none,
    boost::optional<gtsam::Matrix&> H3 = boost::none,
    boost::optional<gtsam::Matrix&> H4 = boost::none) const {

      double eps = 1e-16;
      double pthres = 0.01;

      //double val = (std::abs(v3[0])*std::abs(v2[0]) - std::abs(v3[0])*std::abs(v1[0]))/(std::abs(v4[0])*std::abs(v1[0])+eps);
      //double val = std::abs(v3[0])*(std::abs(v2[0]) - std::abs(v1[0])) - mu_ * (std::abs(v4[0])*std::abs(v1[0])+eps);
      double val = (std::abs(v3[0])+1e-1)*(std::abs(v2[0]) - std::abs(v1[0])) - mu_ * ((std::abs(v4[0])+1e-1)*std::abs(v1[0]));

      //if (val > mu_ && v2[0] > 0) {
      if (val > 0 && v2[0] > 0 && std::abs(v2[0]) > std::abs(v1[0]) && std::abs(v3[0]) > pthres) {
        if (H1) *H1 = gtsam::Matrix11::Zero();
        //if (H2) *H2 = (gtsam::Matrix11() << std::abs(v3[0])/(std::abs(v4[0])*std::abs(v1[0])+eps)).finished();
        if (H2) *H2 = (gtsam::Matrix11() << std::abs(v3[0])+1e-1).finished();
        if (H3) *H3 = gtsam::Matrix11::Zero();
        if (H4) *H4 = gtsam::Matrix11::Zero();
        //return (gtsam::Vector1() << val - mu_).finished();
        return (gtsam::Vector1() << val).finished();
      //} else if (val > mu_ && v2[0] < 0) {
      } else if (val > 0 && v2[0] < 0 && std::abs(v2[0]) > std::abs(v1[0]) && std::abs(v3[0]) > pthres) {
        if (H1) *H1 = gtsam::Matrix11::Zero();
        //if (H2) *H2 = (gtsam::Matrix11() << -std::abs(v3[0])/(std::abs(v4[0])*std::abs(v1[0])+eps)).finished();
        if (H2) *H2 = (gtsam::Matrix11() << -(std::abs(v3[0])+1e-1)).finished();
        if (H3) *H3 = gtsam::Matrix11::Zero();
        if (H4) *H4 = gtsam::Matrix11::Zero();
        //return (gtsam::Vector1() << val - mu_).finished();
        return (gtsam::Vector1() << val).finished();
      } else {
        if (H1) *H1 = gtsam::Matrix11::Zero();
        if (H2) *H2 = gtsam::Matrix11::Zero();
        if (H3) *H3 = gtsam::Matrix11::Zero();
        if (H4) *H4 = gtsam::Matrix11::Zero();
        return (gtsam::Vector1() << 0.0).finished(); 
      }


      /*
      
      //double val = v3[0]*v2[0] - v3[0]*v1[0];// - mu_*(v4[0]*v1[0] + eps);
      double val = (v3[0]*v2[0] - v3[0]*v1[0])/(v4[0]*v1[0]);

      if (val > mu_ && v2[0] > v1[0] && v4[0] > 0) {
        if (H1) *H1 = gtsam::Matrix11::Zero();//(gtsam::Matrix11() << -v3[0] - mu_*v4[0]).finished();
        //if (H2) *H2 = (gtsam::Matrix11() << v3[0]).finished();
        if (H2) *H2 = gtsam::Matrix11::Zero();// (gtsam::Matrix11() << v3[0]/(v4[0]*v1[0] + eps)).finished();
        if (H3) *H3 = gtsam::Matrix11::Zero();//(gtsam::Matrix11() << v2[0] - v1[0]).finished();
        //if (H4) *H4 = (gtsam::Matrix11() << -mu_*v1[0]).finished();
        //if (H4) *H4 = (gtsam::Matrix11() << -v1[0]*(v3[0]*v2[0] - v3[0]*v1[0])/pow((v4[0]*v1[0] + eps),2)).finished();
        if (H4) *H4 = (gtsam::Matrix11() << ((v2[0]-v1[0])*v3[0])/(v1[0]*v4[0]*v4[0])).finished();
        //if (H4) *H4 = (gtsam::Matrix11() << 1/v3[0]).finished();
        //std::cout << v1[0] << " " << v2[0] << " " << v3[0] << " " << v4[0] << " " << val << std::endl;
        return (gtsam::Vector1() << val - mu_).finished();
      } else {
        if (H1) *H1 = gtsam::Matrix11::Zero();
        if (H2) *H2 = gtsam::Matrix11::Zero();
        if (H3) *H3 = gtsam::Matrix11::Zero();
        if (H4) *H4 = gtsam::Matrix11::Zero();
        //std::cout << v1[0] << " " << v2[0] << " " << v3[0] << " " << v4[0] << " " << val << std::endl;
        //std::cout << v1[0] << " " << v2[0] << " " << v3[0] << " " << v4[0] << " " << 0 << std::endl;
        return (gtsam::Vector1() << 0.0).finished(); 
      }

      double lv1 = std::log(v1[0]);
      double lv2 = std::log(v2[0]-v1[0]);
      double lv3 = std::log(v3[0]);
      double lv4 = std::log(v4[0]);
      double lmu = std::log(mu_);

      double Hl11 = 1/v1[0];
      double Hl21 = -3/(v2[0]-v1[0]);
      double Hl22 = 1/(v2[0]-v1[0]);
      double Hl33 = 1/v3[0];
      double Hl44 = 1/v4[0];

      double val = lv2 + lv3 - lv1 - lv4 - lmu;

      if (val > 0) {
        if (H1) *H1 = gtsam::Matrix11::Zero();
        if (H3) *H3 = gtsam::Matrix11::Zero();
        if (H2) *H2 = (gtsam::Matrix11() << Hl22).finished();
        if (H4) *H4 = (gtsam::Matrix11() << -Hl44).finished();
        return (gtsam::Vector1() << val).finished();
      } else {
        if (H1) *H1 = gtsam::Matrix11::Zero();
        if (H2) *H2 = gtsam::Matrix11::Zero();
        if (H3) *H3 = gtsam::Matrix11::Zero();
        if (H4) *H4 = gtsam::Matrix11::Zero();
        return (gtsam::Vector1() << 0.0).finished(); 
      }
      */
  }

  /** number of variables attached to this factor */
  std::size_t size() const {
    return 4;
  }

}; // \class CoulombFactor

} /// namespace gtsam_packing