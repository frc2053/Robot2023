#include <eigen_fix.h>
#include "str/TwoJointArmDynamics.h"
#include <Eigen/LU>
#include <frc/system/NumericalJacobian.h>
#include <iostream>
#include <frc/system/Discretization.h>
#include <frc/StateSpaceUtil.h>
#include <chrono>
#include <wpi/json_serializer.h>
#include <numbers>
#include <fstream>

//GOOD
//Can't figure out how to make NumericalJacobian take ptr to member func
frc::Vectord<6> DynamicsFreeFunc(const frc::Vectord<6>& state, const frc::Vectord<2>& input, TwoJointArmDynamics* arm) {
  return arm->Dynamics(state, input);
}
typedef frc::Vectord<6>(*dynamicsfunctype)(const frc::Vectord<6>& state, const frc::Vectord<2>& input, TwoJointArmDynamics* arm);
//end crazy-ness

TwoJointArmDynamics::TwoJointArmDynamics(
  units::kilogram_t shoulderMass, units::kilogram_t elbowMass, 
  units::meter_t shoulderLength, units::meter_t elbowLength, 
  units::kilogram_square_meter_t shoulderMoi, units::kilogram_square_meter_t elbowMoi,
  units::meter_t cogDistShoulder, units::meter_t cogDistElbow,
  frc::DCMotor shoulderGearbox, frc::DCMotor elbowGearbox,
  double shoulderGearing, double elbowGearing,
  frc::Vectord<6> initialState,
  double lqrPos, double lqrVel, double lqrEffort,
  double qPos, double qVel, double est, double rPos
) : massOfShoulder(shoulderMass.value()), massOfElbow(elbowMass.value()),
    lengthOfShoulder(shoulderLength.value()), lengthOfElbow(elbowLength.value()),
    shoulderMoi(shoulderMoi.value()), elbowMoi(elbowMoi.value()),
    distToCogShoulder(cogDistShoulder.value()), distToCogElbow(cogDistElbow.value()),
    shoulderGearbox(shoulderGearbox), elbowGearbox(elbowGearbox),
    shoulderGearing(shoulderGearing), elbowGearing(elbowGearing),
    lqrQPos(lqrPos), lqrQVel(lqrVel), lqrR(lqrEffort),
    cSystem(frc::LinearSystem<6,2,2>{frc::Matrixd<6,6>::Zero(), frc::Matrixd<6,2>::Zero(), frc::Matrixd<2,6>::Zero(), frc::Matrixd<2,2>::Zero()})
{
  currentState = initialState;
  //CreateLQRLookupTable();
  RecreateModels();

  ekf = std::make_unique<frc::ExtendedKalmanFilterRKDP<6,2,2>>(
    [this](frc::Vectord<6> state, frc::Vectord<2> input) {
      return Dynamics(state, input);
    },
    [this](frc::Vectord<6> state, frc::Vectord<2> input) {
      return UpdateMeasurementState(state, input);
    },
    std::array<double, 6>{qPos, qPos, qVel, qVel, est, est},
    std::array<double, 2>{rPos, rPos},
    dt
  );

  ekf->SetXhat(currentState);
}

void TwoJointArmDynamics::Update(const frc::Vectord<2>& input) {
  currentState = frc::RKDP(
    [this](const frc::Vectord<6>& stateIn, const frc::Vectord<2>& inputVec) {
      return DynamicsReal(stateIn, inputVec);
    },
    currentState,
    input,
    dt
  );

  RecreateModels();
  ekf->Predict(input, dt);
  ekf->Correct(input, cSystem.CalculateY(currentState, input));
}

void TwoJointArmDynamics::UpdateReal(units::radian_t shoulderPos, units::radian_t elbowPos, units::radians_per_second_t shoulderVel, units::radians_per_second_t elbowVel, units::radians_per_second_squared_t shoulderAccel, units::radians_per_second_squared_t elbowAccel)
 {
  currentState = frc::Vectord<6>{shoulderPos.value(), elbowPos.value(), shoulderVel.value(), elbowVel.value(), shoulderAccel.value(), elbowAccel.value()};
  RecreateModels();
}

void TwoJointArmDynamics::RecreateModels() {
  Relinearize(currentState, CalculateFeedForward(currentState));
  frc::Matrixd<2, 4> kMatrix = DesignLQR(cSystem, {lqrQPos, lqrQPos, lqrQVel, lqrQVel}, {lqrR, lqrR}).K();
  frc::Vectord<4> error = desiredState.head(4) - currentState.head(4);
  frc::Vectord<2> feedForwardResults = CalculateFeedForward(desiredState);
  ff = feedForwardResults;
  lqrOutput = (kMatrix * error).cwiseMin(12).cwiseMax(-12);
}

frc::Vectord<2> TwoJointArmDynamics::UpdateMeasurementState(const frc::Vectord<6>& state, const frc::Vectord<2>& input) {
  return input * frc::Vectord<1>{randNorm(gen)};
}

void TwoJointArmDynamics::SetDesiredState(const frc::Vectord<6>& state) {
  desiredState = state;
}

void TwoJointArmDynamics::OverrideCurrentState(const frc::Vectord<6>& newState) {
  currentState = newState;
}

frc::Vectord<6> TwoJointArmDynamics::GetCurrentState() const {
  return currentState;
}

frc::Vectord<6> TwoJointArmDynamics::GetDesiredState() const {
  return desiredState;
}

frc::Vectord<6> TwoJointArmDynamics::GetEKFState() const {
  return ekf->Xhat();
}

frc::Vectord<2> TwoJointArmDynamics::GetFeedForwardVoltage() const {
  return ff;
}

frc::Vectord<2> TwoJointArmDynamics::GetLQROutput() const {
  return lqrOutput;
}

//GOOD
void TwoJointArmDynamics::Relinearize(const frc::Vectord<6>& state, const frc::Vectord<2>& input) {
  cSystem = CreateModel(state, input);
}

//GOOD
frc::LinearSystem<6, 2, 2> TwoJointArmDynamics::CreateModel(const frc::Vectord<6>& state, const frc::Vectord<2>& input) {
  frc::Matrixd<6, 6> A = frc::NumericalJacobianX<6, 6, 2, dynamicsfunctype, TwoJointArmDynamics *&&>(DynamicsFreeFunc, state, input, this);
  frc::Matrixd<6, 2> B = frc::NumericalJacobianU<6, 6, 2, dynamicsfunctype, TwoJointArmDynamics *&&>(DynamicsFreeFunc, state, input, this);
  frc::Matrixd<2, 6> C;
  C << frc::Matrixd<2,2>::Identity(), frc::Matrixd<2, 4>::Zero();
  frc::Matrixd<2,2> D = frc::Matrixd<2,2>::Zero();
  return frc::LinearSystem<6, 2, 2>(A, B, C, D);
}

//GOOD
frc::LinearQuadraticRegulator<4,2> TwoJointArmDynamics::DesignLQR(const frc::LinearSystem<6,2,2>& system, const std::array<double, 4>& qElems, const std::array<double, 2>& rElems) const {
  frc::Matrixd<4, 4> Ar = system.A().block<4, 4>(0,0);
  frc::Matrixd<4, 2> Br = system.B().block<4, 2>(0,0);
  frc::Matrixd<2, 4> Cr = system.C().block<2, 4>(0,0);
  frc::LinearSystem<4, 2, 2> reducedSystem{Ar, Br, Cr, system.D()};
  frc::LinearQuadraticRegulator<4, 2> lqr = frc::LinearQuadraticRegulator<4, 2>(reducedSystem, qElems, rElems, dt);
  return lqr;
}

void TwoJointArmDynamics::CreateLQRLookupTable() {
  //iterate through each xy position in 5cm increments
  //to create lookup table for LQR gains
  for(units::radian_t shoulderAngle = 0_rad; shoulderAngle <= 360_deg; shoulderAngle = shoulderAngle + 1_deg) {
    for(units::radian_t elbowAngle = 0_rad; elbowAngle <= 360_deg; elbowAngle = elbowAngle + 1_deg) {
      frc::Vectord<6> state{shoulderAngle.value(), elbowAngle.value(), 0, 0, 0, 0};
      frc::Vectord<2> control = CalculateFeedForward(state);
      std::cout << "State: " << state << "\n";
      std::cout << "----\n";
      std::cout << "Control: " << control << "\n";
      frc::LinearSystem<6, 2, 2> system = CreateModel(state, control);
      frc::Matrixd<2, 4> resultKMatrix = DesignLQR(system, {lqrQPos, lqrQPos, lqrQVel, lqrQVel}, {lqrR, lqrR}).K();

      frc::Vectord<2> xAndY = CalculateInverseKinematics(frc::Vectord<2>{shoulderAngle.value(), elbowAngle.value()});
      kGainLookupTable.insert(str::ArmCoordinate{units::meter_t{xAndY(0)},units::meter_t{xAndY(1)}}, resultKMatrix);
    }
  }
}

//GOOD
std::tuple<frc::Matrixd<2,2>,frc::Matrixd<2,2>,frc::Vectord<2>> TwoJointArmDynamics::GetDynamicsMatrices(const frc::Vectord<6>& state) const {
  frc::Matrixd<2,2> M = CalculateInertiaMatrix(state({0,1}));
  frc::Matrixd<2,2> C = CalculateCentrifugalCoriolisTerms(state({0,1}), state({2,3}));
  frc::Vectord<2> G = CalculateGravityTorque(state({0,1}), state({2,3}));
  return std::make_tuple(M, C, G);
}

//GOOD
frc::Vectord<6> TwoJointArmDynamics::Dynamics(const frc::Vectord<6>& state, const frc::Vectord<2>& input) const {
  frc::Matrixd<2,2> M;
  frc::Matrixd<2,2> C;
  frc::Vectord<2> G;
  std::tie(M, C, G) = GetDynamicsMatrices(state);

  frc::Vectord<2> omegas = state({2,3});

  frc::Vectord<2> basicTorque = CalculateMotorTorque() * input;
  frc::Vectord<2> backEmfLoss = CalculateBackEMF() * omegas;

  frc::Vectord<2> torque = basicTorque - backEmfLoss;
  frc::Vectord<2> alpha = M.inverse() * (torque - C * omegas - G);
  frc::Vectord<6> stateDot;
  stateDot << omegas, alpha, frc::Vectord<2>{0,0};
  return stateDot;
}

//GOOD
frc::Vectord<6> TwoJointArmDynamics::DynamicsReal(const frc::Vectord<6>& state, const frc::Vectord<2>& input) const {
  frc::Matrixd<2,2> M;
  frc::Matrixd<2,2> C;
  frc::Vectord<2> G;
  std::tie(M, C, G) = GetDynamicsMatrices(state);

  frc::Vectord<2> omegas = state({2,3});

  frc::Vectord<2> basicTorque = CalculateMotorTorque() * input;
  frc::Vectord<2> backEmfLoss = CalculateBackEMF() * omegas;
  frc::Vectord<2> disturbanceTorque = frc::Vectord<2>{0, 2.8};

  frc::Vectord<2> torque = basicTorque - backEmfLoss + disturbanceTorque;
  frc::Vectord<2> alpha = M.inverse() * (torque - C * omegas - G);
  frc::Vectord<6> stateDot;
  stateDot << omegas, alpha, frc::Vectord<2>{0,0};

  return stateDot;
}

//GOOD
frc::Vectord<2> TwoJointArmDynamics::CalculateFeedForward(const frc::Vectord<6>& state, const frc::Vectord<2>& accels) const {
  frc::Matrixd<2,2> M;
  frc::Matrixd<2,2> C;
  frc::Vectord<2> G;
  std::tie(M, C, G) = GetDynamicsMatrices(state);

  frc::Vectord<2> omegas = state({2,3});
  return CalculateMotorTorque().inverse() * (M * accels + C * omegas + G + CalculateBackEMF() * omegas);
}

//GOOD
std::tuple<frc::Vectord<2>,frc::Vectord<2>,frc::Vectord<2>> TwoJointArmDynamics::CalculateForwardKinematics(const frc::Vectord<6>& state) const {
  frc::Vectord<2> thetas = state({0,1});

  frc::Vectord<2> elbowJointPosition;
  elbowJointPosition << lengthOfShoulder * std::cos(thetas(0)), 
                        lengthOfShoulder * std::sin(thetas(0));

  frc::Vectord<2> endPosition;
  endPosition << lengthOfElbow * std::cos(thetas(0) + thetas(1)),
                  lengthOfElbow * std::sin(thetas(0) + thetas(1));

  frc::Vectord<2> endEffector;
  endEffector << elbowJointPosition + endPosition;

  return std::make_tuple(elbowJointPosition, endPosition, endEffector);
}

//GOOD
//Reference: https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
frc::Vectord<2> TwoJointArmDynamics::CalculateInverseKinematics(const frc::Vectord<2>& position, bool fromTop) const {

  double elbowAngle = 0.0;
  double shoulderAngle = 0.0;

  frc::Vectord<2> relativePositon = position;

  bool isFlipped = position(0) < 0.0;

  if(isFlipped) {
    relativePositon(0) = -relativePositon(0);
  }

  if(fromTop) {
    double elbowAngleNumerator = std::pow(relativePositon(0), 2) + std::pow(relativePositon(1), 2) - std::pow(lengthOfShoulder, 2) - std::pow(lengthOfElbow, 2);
    double elbowAngleDenominator = 2 * lengthOfShoulder * lengthOfElbow;
    elbowAngle = -std::acos(elbowAngleNumerator / elbowAngleDenominator);

    double shoulderAngleNumerator = lengthOfElbow * std::sin(elbowAngle);
    double shoulderAngleDenominator = lengthOfShoulder + (lengthOfElbow * std::cos(elbowAngle));
    shoulderAngle = std::atan(relativePositon(1) / relativePositon(0)) - std::atan(shoulderAngleNumerator / shoulderAngleDenominator);
  }
  else {
    double elbowAngleNumerator = std::pow(relativePositon(0), 2) + std::pow(relativePositon(1), 2) - std::pow(lengthOfShoulder, 2) - std::pow(lengthOfElbow, 2);
    double elbowAngleDenominator = 2 * lengthOfShoulder * lengthOfElbow;
    elbowAngle = std::acos(elbowAngleNumerator / elbowAngleDenominator);

    double shoulderAngleNumerator = lengthOfElbow * std::sin(elbowAngle);
    double shoulderAngleDenominator = lengthOfShoulder + lengthOfElbow * std::cos(elbowAngle);
    shoulderAngle = std::atan(relativePositon(1) / relativePositon(0)) - std::atan(shoulderAngleNumerator / shoulderAngleDenominator);
  }

  if(isFlipped) {
    shoulderAngle = std::numbers::pi - shoulderAngle;
    elbowAngle = -elbowAngle;
  }

  frc::Vectord<2> result;
  result << shoulderAngle,
            elbowAngle;
  return result;
}

//GOOD
frc::Matrixd<2,2> TwoJointArmDynamics::CalculateInertiaMatrix(const frc::Vectord<2>& angleVec) const {
  frc::Matrixd<2,2> inertiaMatrix;
  inertiaMatrix << massOfShoulder * std::pow(distToCogShoulder, 2) + massOfElbow * (std::pow(lengthOfShoulder, 2) + std::pow(distToCogElbow, 2)) + shoulderMoi + elbowMoi + 2 * massOfElbow * lengthOfShoulder * distToCogElbow * std::cos(angleVec(1)),
                   massOfElbow * std::pow(distToCogElbow, 2) + elbowMoi + massOfElbow * lengthOfShoulder * distToCogElbow * std::cos(angleVec(1)),
                   massOfElbow * std::pow(distToCogElbow, 2) + elbowMoi + massOfElbow * lengthOfShoulder * distToCogElbow * std::cos(angleVec(1)), 
                   massOfElbow * std::pow(distToCogElbow, 2) + elbowMoi;
  return inertiaMatrix;
}

//GOOD
frc::Matrixd<2,2> TwoJointArmDynamics::CalculateCentrifugalCoriolisTerms(const frc::Vectord<2>& angleVec, const frc::Vectord<2>& velocityVec) const {
  frc::Matrixd<2,2> ccMatrix;
  ccMatrix << -massOfElbow * lengthOfShoulder * distToCogElbow * std::sin(angleVec(1)) * velocityVec(1),
              -massOfElbow * lengthOfShoulder * distToCogElbow * std::sin(angleVec(1)) * (velocityVec(0) + velocityVec(1)),
              massOfElbow * lengthOfShoulder * distToCogElbow * std::sin(angleVec(1)) * velocityVec(0),
              0;
  return ccMatrix;
}

//GOOD
frc::Vectord<2> TwoJointArmDynamics::CalculateGravityTorque(const frc::Vectord<2>& angleVec, const frc::Vectord<2>& velocityVec) const {
  frc::Vectord<2> gravityTorqueMatrix;
  gravityTorqueMatrix << (massOfShoulder * distToCogShoulder + massOfElbow * lengthOfShoulder) * GRAVITY * std::cos(angleVec(0)) + massOfElbow * distToCogElbow * GRAVITY * std::cos(angleVec(0) + angleVec(1)), 
                         massOfElbow * distToCogElbow * GRAVITY * std::cos(angleVec(0) + angleVec(1));
  return gravityTorqueMatrix;
}

//GOOD
frc::Matrixd<2,2> TwoJointArmDynamics::CalculateMotorTorque() const {
  frc::Matrixd<2,2> motorMatrix;
  motorMatrix << shoulderGearing * shoulderGearbox.Kt.value() / shoulderGearbox.R.value(),
                  0,
                  0,
                  elbowGearing * elbowGearbox.Kt.value() / elbowGearbox.R.value();
  return motorMatrix;
}

//GOOD
frc::Matrixd<2,2> TwoJointArmDynamics::CalculateBackEMF() const {
  frc::Matrixd<2,2> backEmfMatrix;
  backEmfMatrix << (std::pow(shoulderGearing, 2) * shoulderGearbox.Kt.value()) / (shoulderGearbox.Kv.value() * shoulderGearbox.R.value()),
                  0,
                  0,
                  (std::pow(elbowGearing, 2) * elbowGearbox.Kt.value()) / (elbowGearbox.Kv.value() * elbowGearbox.R.value());
  backEmfMatrix = backEmfMatrix.cwiseMin(12);
  return backEmfMatrix;
}