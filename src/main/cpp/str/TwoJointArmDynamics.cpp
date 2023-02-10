#include <eigen_fix.h>
#include "str/TwoJointArmDynamics.h"
#include <Eigen/LU>
#include <frc/system/NumericalJacobian.h>
#include <iostream>
#include <frc/system/Discretization.h>
#include <frc/StateSpaceUtil.h>
#include <chrono>

//GOOD
//Can't figure out how to make NumericalJacobian take ptr to member func
frc::Vectord<6> DynamicsFreeFunc(frc::Vectord<6> state, frc::Vectord<2> input, TwoJointArmDynamics* arm) {
  return arm->Dynamics(state, input);
}
typedef frc::Vectord<6>(*dynamicsfunctype)(frc::Vectord<6> state, frc::Vectord<2> input, TwoJointArmDynamics* arm);
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
    cSystem(frc::LinearSystem<6,2,2>{frc::Matrixd<6,6>::Zero(), frc::Matrixd<6,2>::Zero(), frc::Matrixd<2,6>::Zero(), frc::Matrixd<2,2>::Zero()}),
    dSystem(frc::LinearSystem<6,2,2>{frc::Matrixd<6,6>::Zero(), frc::Matrixd<6,2>::Zero(), frc::Matrixd<2,6>::Zero(), frc::Matrixd<2,2>::Zero()})
{
  currentState = initialState;
  //CreateLQRLookupTable();
  RecreateModels();

  ekf = std::make_unique<frc::ExtendedKalmanFilter<6,2,2>>(
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
}

void TwoJointArmDynamics::Update(frc::Vectord<2> input) {
  currentState = frc::RKDP(
    [this](frc::Vectord<6> stateIn, frc::Vectord<2> inputVec) {
      return DynamicsReal(stateIn, inputVec);
    },
    currentState,
    input,
    dt
  );

  RecreateModels();
  //ekf->Predict(input, dt);
  //ekf->Correct(input, currentState.head(2));
}

void TwoJointArmDynamics::RecreateModels() {
  Relinearize(currentState, CalculateFeedForward(currentState));
  frc::Matrixd<2, 4> kMatrix = DesignLQR(cSystem, {lqrQPos, lqrQPos, lqrQVel, lqrQVel}, {lqrR, lqrR}).K();
  frc::Vectord<4> error = desiredState.head(4) - currentState.head(4);
  ff = CalculateFeedForward(desiredState) + (kMatrix * error).cwiseMin(12).cwiseMax(-12);
}

frc::Vectord<2> TwoJointArmDynamics::UpdateMeasurementState(frc::Vectord<6> state, frc::Vectord<2> input) {
  return dSystem.C() * state + dSystem.D() * input + frc::Vectord<2>{randNorm(gen), randNorm(gen)};
}

void TwoJointArmDynamics::SetDesiredState(frc::Vectord<6> state) {
  desiredState = state;
}

frc::Vectord<6> TwoJointArmDynamics::GetCurrentState() const {
  return currentState;
}

frc::Vectord<6> TwoJointArmDynamics::GetEKFState() const {
  return ekf->Xhat();
}

frc::Vectord<2> TwoJointArmDynamics::GetVoltagesToApply() const {
  return ff;
}

//GOOD
void TwoJointArmDynamics::Relinearize(frc::Vectord<6> state, frc::Vectord<2> input) {
  cSystem = CreateModel(state, input);
  frc::Matrixd<6, 6> dA = frc::Matrixd<6, 6>::Zero();
  frc::Matrixd<6, 2> dB = frc::Matrixd<6, 2>::Zero();
  frc::DiscretizeAB(cSystem.A(), cSystem.B(), dt, &dA, &dB);
  frc::LinearSystem<6, 2, 2> discretizedSystem{dA, dB, cSystem.C(), cSystem.D()};
  dSystem = discretizedSystem;
}

//GOOD
frc::LinearSystem<6, 2, 2> TwoJointArmDynamics::CreateModel(frc::Vectord<6> state, frc::Vectord<2> input) {
  frc::Matrixd<6, 6> A = frc::NumericalJacobianX<6, 6, 2, dynamicsfunctype, TwoJointArmDynamics *&&>(DynamicsFreeFunc, state, input, this);
  frc::Matrixd<6, 2> B = frc::NumericalJacobianU<6, 6, 2, dynamicsfunctype, TwoJointArmDynamics *&&>(DynamicsFreeFunc, state, input, this);
  frc::Matrixd<2, 6> C;
  C << frc::Matrixd<2,2>::Identity(), frc::Matrixd<2, 4>::Zero();
  frc::Matrixd<2,2> D = frc::Matrixd<2,2>::Zero();
  return frc::LinearSystem<6, 2, 2>(A, B, C, D);
}

//GOOD
frc::LinearQuadraticRegulator<4,2> TwoJointArmDynamics::DesignLQR(const frc::LinearSystem<6,2,2>& system, std::array<double, 4> qElems, std::array<double, 2> rElems) const {
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
  for(units::inch_t r = 70_in; r >= 1_in; r = r - 5_in) {
    for(units::degree_t theta = 0_deg; theta <= 180_deg; theta = theta + 5_deg) {
      units::meter_t x = r * units::math::sin(theta);
      units::meter_t y = r * units::math::cos(theta);
      frc::Vectord<2> angles = CalculateInverseKinematics(frc::Vectord<2>{x.value(),y.value()});
      std::cout << "Designing LQR for state (" << x.value() << "," << y.value() << ")\n";
      frc::Vectord<6> state{angles(0), angles(1), 0, 0, 0, 0};
      frc::Vectord<2> control = CalculateFeedForward(state);
      std::cout << "State: " << state << "\n";
      std::cout << "----\n";
      std::cout << "Control: " << control << "\n";
      frc::LinearSystem<6, 2, 2> system = CreateModel(state, control);
      frc::Matrixd<2, 4> resultKMatrix = DesignLQR(system, {lqrQPos, lqrQPos, lqrQVel, lqrQVel}, {lqrR, lqrR}).K();
      kGainLookupTable.insert(str::ArmCoordinate{x,y}, resultKMatrix);
    }
  }
}

//GOOD
std::tuple<frc::Matrixd<2,2>,frc::Matrixd<2,2>,frc::Vectord<2>> TwoJointArmDynamics::GetDynamicsMatrices(frc::Vectord<6> state) const {
  frc::Matrixd<2,2> M = CalculateInertiaMatrix(state({0,1}));
  frc::Matrixd<2,2> C = CalculateCentrifugalCoriolisTerms(state({0,1}), state({2,3}));
  frc::Vectord<2> G = CalculateGravityTorque(state({0,1}), state({2,3}));
  return std::make_tuple(M, C, G);
}

//GOOD
frc::Vectord<6> TwoJointArmDynamics::Dynamics(frc::Vectord<6> state, frc::Vectord<2> input) const {
  frc::Matrixd<2,2> M;
  frc::Matrixd<2,2> C;
  frc::Vectord<2> G;
  std::tie(M, C, G) = GetDynamicsMatrices(state);

  frc::Vectord<2> omegas = state({2,3});

  frc::Vectord<2> basicTorque = CalculateMotorTorque() * input;
  //basicTorque = basicTorque + state({4,5});
  frc::Vectord<2> backEmfLoss = CalculateBackEMF() * omegas;

  frc::Vectord<2> torque = basicTorque - backEmfLoss;
  frc::Vectord<2> alpha = M.inverse() * (torque - C * omegas - G);
  frc::Vectord<6> stateDot;
  stateDot << omegas, alpha, frc::Vectord<2>{0,0};
  return stateDot;
}

//GOOD
frc::Vectord<6> TwoJointArmDynamics::DynamicsReal(frc::Vectord<6> state, frc::Vectord<2> input) const {
  frc::Matrixd<2,2> M;
  frc::Matrixd<2,2> C;
  frc::Vectord<2> G;
  std::tie(M, C, G) = GetDynamicsMatrices(state);

  frc::Vectord<2> omegas = state({2,3});

  frc::Vectord<2> basicTorque = CalculateMotorTorque() * input;
  frc::Vectord<2> backEmfLoss = CalculateBackEMF() * omegas;
  frc::Vectord<2> disturbanceTorque = frc::Vectord<2>{0, 0};//frc::Vectord<2>{150, -90};

  frc::Vectord<2> torque = basicTorque - backEmfLoss + disturbanceTorque;
  frc::Vectord<2> alpha = M.inverse() * (torque - C * omegas - G);
  frc::Vectord<6> stateDot;
  stateDot << omegas, alpha, frc::Vectord<2>{0,0};
  return stateDot;
}

//GOOD
frc::Vectord<2> TwoJointArmDynamics::CalculateFeedForward(frc::Vectord<6> state, frc::Vectord<2> accels) const {
  frc::Matrixd<2,2> M;
  frc::Matrixd<2,2> C;
  frc::Vectord<2> G;
  std::tie(M, C, G) = GetDynamicsMatrices(state);

  frc::Vectord<2> omegas = state({2,3});
  return CalculateMotorTorque().inverse() * (M * accels + C * omegas + G + CalculateBackEMF() * omegas);
}

//GOOD
std::tuple<frc::Vectord<2>,frc::Vectord<2>,frc::Vectord<2>> TwoJointArmDynamics::CalculateForwardKinematics(frc::Vectord<6> state) const {
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
frc::Vectord<2> TwoJointArmDynamics::CalculateInverseKinematics(frc::Vectord<2> position, bool invert) const {
  double elbowAngle = std::acos((std::pow(position(0), 2) + std::pow(position(1), 2) - (std::pow(lengthOfShoulder, 2) + std::pow(lengthOfElbow, 2))) / (2 * lengthOfShoulder * lengthOfElbow));
  if(invert) {
    elbowAngle = -elbowAngle;
  }
  double shoulderAngle = std::atan2(position(1), position(0)) - std::atan2(lengthOfElbow * std::sin(elbowAngle), lengthOfShoulder + lengthOfElbow * std::cos(elbowAngle));
  frc::Vectord<2> result;
  result << shoulderAngle,
            elbowAngle;
  return result;
}

//GOOD
frc::Matrixd<2,2> TwoJointArmDynamics::CalculateInertiaMatrix(frc::Vectord<2> angleVec) const {
  frc::Matrixd<2,2> inertiaMatrix;
  inertiaMatrix << massOfShoulder * std::pow(distToCogShoulder, 2) + massOfElbow * (std::pow(lengthOfShoulder, 2) + std::pow(distToCogElbow, 2)) + shoulderMoi + elbowMoi + 2 * massOfElbow * lengthOfShoulder * distToCogElbow * std::cos(angleVec(0)),
                   massOfElbow * std::pow(distToCogElbow, 2) + elbowMoi + massOfElbow * lengthOfShoulder * distToCogElbow * std::cos(angleVec(1)),
                   massOfElbow * std::pow(distToCogElbow, 2) + elbowMoi + massOfElbow * lengthOfShoulder * distToCogElbow * std::cos(angleVec(1)), 
                   massOfElbow * std::pow(distToCogElbow, 2) + elbowMoi;
  return inertiaMatrix;
}

//GOOD
frc::Matrixd<2,2> TwoJointArmDynamics::CalculateCentrifugalCoriolisTerms(frc::Vectord<2> angleVec, frc::Vectord<2> velocityVec) const {
  frc::Matrixd<2,2> ccMatrix;
  ccMatrix << -massOfElbow * lengthOfShoulder * distToCogElbow * std::sin(angleVec(1)) * velocityVec(1),
              -massOfElbow * lengthOfShoulder * distToCogElbow * std::sin(angleVec(1)) * (velocityVec(0) + velocityVec(1)),
              massOfElbow * lengthOfShoulder * distToCogElbow * std::sin(angleVec(1)) * velocityVec(0),
              0;
  return ccMatrix;
}

//GOOD
frc::Vectord<2> TwoJointArmDynamics::CalculateGravityTorque(frc::Vectord<2> angleVec, frc::Vectord<2> velocityVec) const {
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
  //std::cout << "backemf: " << backEmfMatrix << "\n";
  return backEmfMatrix;
}