#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace str {
  class Field {
  public:
    static Field& GetInstance() {
      static Field instance;
      return instance;
    };
    Field(Field const&) = delete;
    void operator=(Field const&) = delete;

    void SetRobotPosition(const frc::Pose2d& newPosition);
    void SetObjectPosition(std::string_view object_name, const frc::Pose2d& newPosition);
    void DrawTraj(std::string name, const frc::Trajectory& traj);
    frc::Field2d* GetField() {
      return &field;
    };

  private:
    Field() {
    };
    frc::Field2d field;
  };
}   