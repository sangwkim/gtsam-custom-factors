/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * 
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file     gtsam_example.h
 * @brief    Example wrapper interface file for Python
 * @author   Varun Agrawal
 */

// This is an interface file for automatic Python wrapper generation.
// See gtsam.h for full documentation and more examples.

#include <cpp/greeting.h>
#include <cpp/BetweenFactorPose3_.h>
#include <cpp/InequalityFactor.h>
#include <cpp/StiffnessFactor.h>
#include <cpp/StiffnessRatioFactor.h>
#include <cpp/CoulombFactor.h>
#include <cpp/CoulombFactor2.h>
#include <cpp/EnergyWOFFactor.h>
#include <cpp/EnergyWFFactor.h>
#include <cpp/LenPenFactor.h>
#include <cpp/LenPenFactor_2.h>
#include <cpp/LenSlipFactor.h>
#include <cpp/ConsistentDeformFactor.h>
#include <cpp/ContactMotionFactor.h>
#include <cpp/PoseOdometryFactor.h>
#include <cpp/PenetrationFactor.h>
#include <cpp/PenetrationFactor_2.h>
#include <cpp/GPSPose2Factor.h>
#include <cpp/StickingContactFactor.h>
#include <cpp/StickingContactFactor_3D.h>
#include <cpp/TactileTransformFactor.h>
#include <cpp/TactileTransformFactor_3D.h>
#include <cpp/ContactPlaneFactor.h>
#include <cpp/ContactPlaneFactor_3D.h>
#include <cpp/PhysicsFactor.h>
#include <cpp/PhysicsFactor_v2.h>
#include <cpp/PhysicsFactor_v2_2.h>
#include <cpp/PhysicsFactor_v3.h>
#include <cpp/FrictionConeFactor.h>
#include <cpp/InsideFrictionConeFactor.h>
#include <cpp/AlignFactor.h>
#include <cpp/ContactLineParallelFactor.h>
#include <cpp/FixedRotationFactor.h>
#include <cpp/ClineCenterFactor.h>

// The namespace should be the same as in the c++ source code.
namespace gtsam_packing {


virtual class BetweenFactorPose3_ : gtsam::NoiseModelFactor {
  BetweenFactorPose3_(size_t key1, size_t key2, gtsam::Pose3 p,
    const gtsam::noiseModel::Base* model);
};

virtual class InequalityFactor : gtsam::NoiseModelFactor {
  InequalityFactor(size_t key1, double b,
    const gtsam::noiseModel::Base* model);
};

virtual class StiffnessFactor : gtsam::NoiseModelFactor {
  StiffnessFactor(size_t key1, size_t key2, size_t key3, size_t key4, size_t key5, bool zeroJac);
};

virtual class StiffnessRatioFactor : gtsam::NoiseModelFactor {
  StiffnessRatioFactor(size_t key1, size_t key2, size_t key3, size_t key4, size_t key5, size_t key6,
    const gtsam::Vector6& v_nominal,
    const gtsam::noiseModel::Base* model);
};

virtual class CoulombFactor : gtsam::NoiseModelFactor {
  CoulombFactor(size_t key1, size_t key2, size_t key3, size_t key4, double mu,
    const gtsam::noiseModel::Base* model);
};

virtual class CoulombFactor2 : gtsam::NoiseModelFactor {
  CoulombFactor2(size_t key1,
    gtsam::Pose3 O_0,
    gtsam::Pose3 G_0,
    gtsam::Pose3 G_i,
    gtsam::Pose3 Q_i,
    double E_N,
    double l_N,
    double MU,
    gtsam::Vector6 S,    
    const gtsam::noiseModel::Base* model);
};

virtual class LenSlipFactor : gtsam::NoiseModelFactor {
  LenSlipFactor(size_t ks1, size_t ks2, size_t ks3,
    const gtsam::noiseModel::Base* model);
};

virtual class LenPenFactor : gtsam::NoiseModelFactor {
  LenPenFactor(size_t kp1, size_t kp2, size_t kp3,
    const gtsam::noiseModel::Base* model);
};

virtual class LenPenFactor_2 : gtsam::NoiseModelFactor {
  LenPenFactor_2(size_t kp1, size_t kp2, size_t kp3, size_t kp4,
    const gtsam::noiseModel::Base* model);
};

virtual class EnergyWFFactor : gtsam::NoiseModelFactor {
  EnergyWFFactor(size_t kwf1, size_t kwf2, size_t kwf3, size_t kwf4, size_t kwf5,
    const gtsam::Vector6& swf,
    const gtsam::noiseModel::Base* model);
};

virtual class EnergyWOFFactor : gtsam::NoiseModelFactor {
  EnergyWOFFactor(size_t kwof1, size_t kwof2, size_t kwof3, size_t kwof4, size_t kwof5,
    const gtsam::Vector6& swof,
    const gtsam::noiseModel::Base* model);
};

virtual class ConsistentDeformFactor : gtsam::NoiseModelFactor {
  ConsistentDeformFactor(size_t key1, size_t key2, size_t key3, size_t key4, size_t key5, size_t key6,
    const gtsam::noiseModel::Base* model);
};

virtual class ContactMotionFactor : gtsam::NoiseModelFactor {
  ContactMotionFactor(size_t key1, size_t key2, size_t key3, size_t key4,
    const gtsam::noiseModel::Base* model, bool zeroJac);
};

virtual class PoseOdometryFactor : gtsam::NoiseModelFactor {
  PoseOdometryFactor(size_t key1, size_t key2, size_t key3,
    const gtsam::noiseModel::Base* model, bool zeroJac);
};

virtual class PenetrationFactor : gtsam::NoiseModelFactor {
  PenetrationFactor(size_t key1, size_t key2,
    double cost_sigma, double eps);
};

virtual class PenetrationFactor_2 : gtsam::NoiseModelFactor {
  PenetrationFactor_2(size_t key1, size_t key2, size_t key3,
    double cost_sigma, double eps);
};

virtual class ClineCenterFactor : gtsam::NoiseModelFactor {
  ClineCenterFactor(size_t key1,
    const gtsam::noiseModel::Base* model);
};

virtual class FixedRotationFactor : gtsam::NoiseModelFactor {
  FixedRotationFactor(size_t key1, size_t key2, size_t key3, size_t key4,
    const gtsam::Pose2& m_1, const gtsam::Pose2& m_2,
    const gtsam::noiseModel::Base* model);
};

virtual class StickingContactFactor : gtsam::NoiseModelFactor {
  StickingContactFactor(size_t key1, size_t key2, size_t key3, size_t key4,
    const gtsam::noiseModel::Base* model);
};

virtual class StickingContactFactor_3D : gtsam::NoiseModelFactor {
  StickingContactFactor_3D(size_t key1, size_t key2, size_t key3, size_t key4,
    const gtsam::noiseModel::Base* model, bool zeroJac);
};

virtual class TactileTransformFactor : gtsam::NoiseModelFactor {
  TactileTransformFactor(size_t key1, size_t key2, size_t key3, size_t key4,
  	const gtsam::Pose2& m, const gtsam::noiseModel::Base* model);
};

virtual class TactileTransformFactor_3D : gtsam::NoiseModelFactor {
  TactileTransformFactor_3D(size_t key1, size_t key2, size_t key3, size_t key4,
    const gtsam::Pose3& m, const gtsam::noiseModel::Base* model, bool zeroJac);
};

virtual class ContactPlaneFactor : gtsam::NoiseModelFactor {
  ContactPlaneFactor(size_t key1, size_t key2, const gtsam::Vector1& m, const gtsam::noiseModel::Base* model);
};

virtual class ContactPlaneFactor_3D : gtsam::NoiseModelFactor {
  ContactPlaneFactor_3D(size_t key1, size_t key2, const gtsam::noiseModel::Base* model);
};

virtual class PhysicsFactor : gtsam::NoiseModelFactor {
  PhysicsFactor(size_t key1, size_t key2, size_t key3, size_t key4,
  	const gtsam::Pose2& m, const gtsam::noiseModel::Base* model);
};

virtual class PhysicsFactor_v2 : gtsam::NoiseModelFactor {
  PhysicsFactor_v2(size_t key1, size_t key2, size_t key3, size_t key4,
    const gtsam::Pose2& m, const gtsam::noiseModel::Base* model);
};

virtual class PhysicsFactor_v2_2 : gtsam::NoiseModelFactor {
  PhysicsFactor_v2_2(size_t key1, size_t key2, size_t key3, size_t key4,
    const gtsam::Pose2& m, const gtsam::noiseModel::Base* model);
};

virtual class PhysicsFactor_v3 : gtsam::NoiseModelFactor {
  PhysicsFactor_v3(size_t key1, size_t key2, size_t key3, size_t key4, size_t key5,
    const gtsam::Pose2& m, const gtsam::noiseModel::Base* model);
};

virtual class FrictionConeFactor : gtsam::NoiseModelFactor {
  FrictionConeFactor(size_t key1, size_t key2, size_t key3, size_t key4, size_t key5, size_t key6,
    const gtsam::Pose2& m, const gtsam::noiseModel::Base* model);
};

virtual class InsideFrictionConeFactor : gtsam::NoiseModelFactor {
  InsideFrictionConeFactor(size_t key1, size_t key2,
    const gtsam::noiseModel::Base* model);
};

virtual class GPSPose2Factor : gtsam::NoiseModelFactor {
  GPSPose2Factor(size_t poseKey, const gtsam::Point2& m, gtsam::noiseModel::Base* model);
};

virtual class AlignFactor : gtsam::NoiseModelFactor {
  AlignFactor(size_t key1, const gtsam::noiseModel::Base* model);
};

virtual class ContactLineParallelFactor : gtsam::NoiseModelFactor {
  ContactLineParallelFactor(size_t key1, size_t key2, const gtsam::noiseModel::Base* model);
};

class Greeting {
  Greeting();
  void sayHello() const;
  gtsam::Rot3 invertRot3(gtsam::Rot3 rot) const;
  void sayGoodbye() const;
};

}  // namespace gtsam_example
