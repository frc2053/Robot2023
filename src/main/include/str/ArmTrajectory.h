#pragma once

#include <frc/EigenCore.h>
#include <units/time.h>
#include <units/math.h>
#include <cmath>
#include <algorithm>

struct ArmTrajectoryParams {
  frc::Vectord<2> initialState;
  frc::Vectord<2> finalState;
};

class ArmTrajectory {
public:
  ArmTrajectory(ArmTrajectoryParams parameters) : params{parameters} {
    points.push_back(params.initialState);
    points.push_back(params.finalState);
  };
  ArmTrajectoryParams GetParams() const {
    return params;
  };
  bool IsGenerated() {
    return totalTime > 0.0_s && points.size() > 2;
  };
  void SetPoints(units::second_t time, const std::vector<frc::Vectord<2>>& newPoints) {
    totalTime = time;
    points = newPoints;
  };
  units::second_t GetTotalTime() const {
    return totalTime;
  };
  std::vector<frc::Vectord<2>> GetPoints() const {
    return points;
  };
  frc::Matrixd<2, 3> Sample(units::second_t time) {
    units::second_t dt = totalTime / (points.size() - 1);

    int prevIndex = units::math::floor(time / dt);
    int nextIndex = units::math::ceil(time / dt);
    if(nextIndex == prevIndex) {
      nextIndex++;
    }
    int secondPrevIndex = prevIndex - 1;
    int secondNextIndex = nextIndex + 1;

    prevIndex = std::clamp(prevIndex, 0, (int)points.size() - 1);
    nextIndex = std::clamp(nextIndex, 0, (int)points.size() - 1);

    secondPrevIndex = std::clamp(secondPrevIndex, 0, (int)points.size() - 1);
    secondNextIndex = std::clamp(secondNextIndex, 0, (int)points.size() - 1);

    double pos0 = std::lerp(points[prevIndex](0), points[nextIndex](0), (std::remainder(time.value(), dt.value())) / dt.value());
    double pos1 = std::lerp(points[prevIndex](1), points[nextIndex](1), (std::remainder(time.value(), dt.value())) / dt.value());

    double vel0 = points[nextIndex](0) - points[prevIndex](0) / dt.value();
    double vel1 = points[nextIndex](1) - points[prevIndex](1) / dt.value();

    double accel0 = 0;
    double accel1 = 0;
    if((std::remainder(time.value(), dt.value())) / dt.value() < 0.5) {
      double prevVel0 = (points[prevIndex](0) - points[secondPrevIndex](0)) / dt.value();
      double prevVel1 = (points[prevIndex](1) - points[secondPrevIndex](1)) / dt.value();
      accel0 = (vel0 - prevVel0) / dt.value();
      accel1 = (vel1 - prevVel1) / dt.value();
    } else {
      double nextVel0 = (points[secondNextIndex](0) - points[nextIndex](0)) / dt.value();
      double nextVel1 = (points[secondNextIndex](1) - points[nextIndex](1)) / dt.value();
      accel0 = (nextVel0 - vel0) / dt.value();
      accel1 = (nextVel1 - vel1) / dt.value();
    }

    frc::Matrixd<2, 3> result;
    result << pos0, vel0, accel0,
              pos1, vel1, accel1;

    return result;
  };
private:
  ArmTrajectoryParams params;
  units::second_t totalTime{0.0};
  std::vector<frc::Vectord<2>> points;
};