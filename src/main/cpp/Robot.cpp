#include "Robot.h"
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <iostream>
#include <frc/simulation/RoboRioSim.h>
#include <wpinet/PortForwarder.h>
#include <str/GenerateArmTrajectoryCacheRequest.h>

void Robot::RobotInit() {
  std::cout << std::boolalpha;

  GenerateArmTrajectoryCacheFile();

  wpi::PortForwarder::GetInstance().Add(5800, "10.20.53.11", 5800);
  wpi::PortForwarder::GetInstance().Add(1181, "10.20.53.11", 1181);
  wpi::PortForwarder::GetInstance().Add(1182, "10.20.53.11", 1182);
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  robot.ConfigureBindings();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  units::volt_t totalVolts = str::PDP::GetInstance().GetBatteryVoltageWithLoad();
  frc::sim::RoboRioSim::SetVInVoltage(totalVolts);
  str::PDP::GetInstance().SetInputVoltage(totalVolts);
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

void Robot::AutonomousInit() {
  autonomousCommand = robot.GetAutonomousCommand();

  if(autonomousCommand) {
    autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
  if(autonomousCommand) {
    autonomousCommand->Cancel();
  }
  robot.SetDriveAsDefault();
}

void Robot::TeleopPeriodic() {
}

void Robot::TestPeriodic() {
}

void Robot::SimulationInit() {
}

void Robot::SimulationPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
