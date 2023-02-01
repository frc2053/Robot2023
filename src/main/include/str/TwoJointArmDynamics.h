#pragma once

#include <frc/EigenCore.h>
#include <units/mass.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <frc/system/plant/DCMotor.h>
#include <tuple>
#include <frc/system/LinearSystem.h>
#include <frc/estimator/ExtendedKalmanFilter.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <random>
#include <memory>
#include "str/interpolating_map_xy.h"

class TwoJointArmDynamics {
public:
  TwoJointArmDynamics(
    units::kilogram_t shoulderMass, units::kilogram_t elbowMass,
    units::meter_t shoulderLength, units::meter_t elbowLength,
    units::kilogram_square_meter_t shoulderMoi, units::kilogram_square_meter_t elbowMoi,
    units::meter_t cogDistShoulder, units::meter_t cogDistElbow,
    frc::DCMotor shoulderGearbox, frc::DCMotor elbowGearbox,
    double shoulderGearing, double elbowGearing,
    frc::Vectord<6> initialState,
    double lqrPos, double lqrVel, double lqrEffort,
    double qPos, double qVel, double est, double rPos
  );

  //LOOPS
  void Update(frc::Vectord<2> input);
  void RecreateModels();

  //KALMAN NOISY
  frc::Vectord<2> UpdateMeasurementState(frc::Vectord<6> state, frc::Vectord<2> input);

  //ACESSESSORS
  void SetDesiredState(frc::Vectord<6> state);
  frc::Vectord<6> GetCurrentState() const;
  frc::Vectord<6> GetEKFState() const;
  frc::Vectord<2> GetVoltagesToApply() const;

  //RELINEARIZATION
  void Relinearize(frc::Vectord<6> state, frc::Vectord<2> input);
  frc::LinearSystem<6, 2, 2> CreateModel(frc::Vectord<6> state, frc::Vectord<2> input);
  frc::LinearQuadraticRegulator<4,2> DesignLQR(frc::LinearSystem<6,2,2> system, std::array<double, 4> qElems, std::array<double, 2> rElems) const;
  void CreateLQRLookupTable();

  //DYNAMICS
  std::tuple<frc::Matrixd<2,2>,frc::Matrixd<2,2>,frc::Vectord<2>> GetDynamicsMatrices(frc::Vectord<6> state) const;
  frc::Vectord<6> Dynamics(frc::Vectord<6> state, frc::Vectord<2> input) const;
  frc::Vectord<6> DynamicsReal(frc::Vectord<6> state, frc::Vectord<2> input) const;
  frc::Vectord<2> CalculateFeedForward(frc::Vectord<6> state, frc::Vectord<2> accels = frc::Vectord<2>{0,0}) const;

  //HELPERS
  std::tuple<frc::Vectord<2>,frc::Vectord<2>,frc::Vectord<2>> CalculateForwardKinematics(frc::Vectord<6> state) const;
  frc::Vectord<2> CalculateInverseKinematics(frc::Vectord<2> position, bool invert = false) const;

  frc::Matrixd<2,2> CalculateInertiaMatrix(frc::Vectord<2> angleVec) const;
  frc::Matrixd<2,2> CalculateCentrifugalCoriolisTerms(frc::Vectord<2> angleVec, frc::Vectord<2> velocityVec) const;
  frc::Vectord<2> CalculateGravityTorque(frc::Vectord<2> angleVec, frc::Vectord<2> velocityVec) const;
  frc::Matrixd<2,2> CalculateMotorTorque() const;
  frc::Matrixd<2,2> CalculateBackEMF() const;

  //ARM PROPERTIES
  const double massOfShoulder;
  const double massOfElbow;
  const double lengthOfShoulder;
  const double lengthOfElbow;
  const double shoulderMoi;
  const double elbowMoi;
  const double distToCogShoulder;
  const double distToCogElbow;
  const frc::DCMotor shoulderGearbox;
  const frc::DCMotor elbowGearbox;
  const double shoulderGearing;
  const double elbowGearing;
  const double GRAVITY{9.81};
  const units::second_t dt{0.02};

  //CONTROLLER PROPERTIES
  const double lqrQPos;
  const double lqrQVel;
  const double lqrR;

private:
  //non const
  frc::LinearSystem<6,2,2> cSystem;
  frc::LinearSystem<6,2,2> dSystem;
  frc::Vectord<6> desiredState{frc::Vectord<6>::Zero()};
  frc::Vectord<6> currentState{frc::Vectord<6>::Zero()};
  frc::Vectord<2> ff{frc::Vectord<2>::Zero()};
  std::unique_ptr<frc::ExtendedKalmanFilter<6, 2, 2>> ekf;
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<> randNorm{0, 0.01};
  //str::interpolating_map_xy<frc::Matrixd<2, 4>> kGainLookupTable;
};