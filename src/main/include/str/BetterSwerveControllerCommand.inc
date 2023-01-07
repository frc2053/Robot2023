// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>

#include "str/BetterSwerveControllerCommand.h"
#include <iostream>

namespace frc2 {

template <size_t NumModules>
BetterSwerveControllerCommand<NumModules>::BetterSwerveControllerCommand(
    frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<frc::Rotation2d()> desiredRotation,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    std::initializer_list<Subsystem*> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_desiredRotation(std::move(desiredRotation)),
      m_outputStates(output) {
  this->AddRequirements(requirements);
}

template <size_t NumModules>
BetterSwerveControllerCommand<NumModules>::BetterSwerveControllerCommand(
    frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    std::initializer_list<Subsystem*> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_outputStates(output) {
  this->AddRequirements(requirements);
}

template <size_t NumModules>
BetterSwerveControllerCommand<NumModules>::BetterSwerveControllerCommand(
    frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<frc::Rotation2d()> desiredRotation,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    std::span<Subsystem* const> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_desiredRotation(std::move(desiredRotation)),
      m_outputStates(output) {
  this->AddRequirements(requirements);
}

template <size_t NumModules>
BetterSwerveControllerCommand<NumModules>::BetterSwerveControllerCommand(
    frc::Trajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    std::span<Subsystem* const> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_outputStates(output) {
  this->AddRequirements(requirements);
}

template <size_t NumModules>
void BetterSwerveControllerCommand<NumModules>::Initialize() {
  if (m_desiredRotation == nullptr) {
    m_desiredRotation = [&] {
      return m_trajectory.States().back().pose.Rotation();
    };
  }
  m_timer.Reset();
  m_timer.Start();
  m_controller.SetTolerance(frc::Pose2d{frc::Translation2d{2_in, 2_in}, frc::Rotation2d{2_deg}});
}

template <size_t NumModules>
void BetterSwerveControllerCommand<NumModules>::Execute() {
  auto curTime = m_timer.Get();
  auto m_desiredState = m_trajectory.Sample(curTime);

  auto targetChassisSpeeds =
      m_controller.Calculate(m_pose(), m_desiredState, m_desiredRotation());
  auto targetModuleStates =
      m_kinematics.ToSwerveModuleStates(targetChassisSpeeds);

  m_outputStates(targetModuleStates);
}

template <size_t NumModules>
void BetterSwerveControllerCommand<NumModules>::End(bool interrupted) {
  m_timer.Stop();
}

template <size_t NumModules>
bool BetterSwerveControllerCommand<NumModules>::IsFinished() {
  frc::Pose2d endPose = m_trajectory.States().back().pose;
  frc::Pose2d currentPose = m_pose();

  bool atLastPoint = units::math::abs(currentPose.X() - endPose.X()) < 2_in && 
    units::math::abs(currentPose.Y() - endPose.Y()) < 2_in && 
    units::math::abs(currentPose.Rotation().Degrees() - endPose.Rotation().Degrees()) < 2_deg;

  bool atSetpoint = m_controller.AtReference();

  bool timeElapsed = m_timer.HasElapsed(m_trajectory.TotalTime());

  // std::cout << "atLastPoint: " << atLastPoint << "\n";
  // std::cout << "atSetpoint: " << atSetpoint << "\n";
  // std::cout << "timeElapsed: " << timeElapsed << "\n";

  return atLastPoint && atSetpoint && timeElapsed;
}

}  // namespace frc2