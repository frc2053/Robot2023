#pragma once

#include <frc/EigenCore.h>
#include <units/time.h>
#include <units/math.h>
#include <cmath>
#include <functional>
#include <algorithm>

struct ArmTrajectoryParams {
  frc::Vectord<2> initialState{0,0};
  frc::Vectord<2> finalState{0,0};

  bool operator==(const ArmTrajectoryParams& other) {
    return ((this->initialState - other.initialState).norm() < 1e-5) && ((this->finalState - other.finalState).norm() < 1e-5);
  }
};

namespace std {
  template <>
  struct hash<ArmTrajectoryParams>
  {
    std::size_t operator()(const ArmTrajectoryParams& k) const
    {
      std::size_t res = 17;

      res = res * 31 + std::hash<double>()(k.initialState(0));
      res = res * 31 + std::hash<double>()(k.initialState(1));
      res = res * 31 + std::hash<double>()(k.finalState(0));
      res = res * 31 + std::hash<double>()(k.finalState(1));

      return res;
    }
  };
}

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
  frc::Vectord<6> Sample(units::second_t time) const {
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

    double vel0 = (points[nextIndex](0) - points[prevIndex](0)) / dt.value();
    double vel1 = (points[nextIndex](1) - points[prevIndex](1)) / dt.value();

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

    frc::Vectord<6> result;
    result << pos0, pos1, vel0, vel1, accel0, accel1;

    return result;
  };

  static ArmTrajectory DefaultTraj() {
    ArmTrajectoryParams params;
    params.initialState = frc::Vectord<2>{0.248886761314, -1.8326};
    params.finalState = frc::Vectord<2>{2.2689280275926285, 0.5235987755982988};

    ArmTrajectory traj(params);

    traj.SetPoints(0.8579333123188765_s, {
      frc::Vectord<2>{0.248886761314, -1.8326},
      frc::Vectord<2>{0.2509644923284381, -1.830444282775462}, 
      frc::Vectord<2>{0.25918089352824747, -1.821821413929597}, 
      frc::Vectord<2>{0.2754735913617155, -1.804575676307619}, 
      frc::Vectord<2>{0.3017081152100597, -1.776551352781537}, 
      frc::Vectord<2>{0.33961558003494036, -1.7355927262695017}, 
      frc::Vectord<2>{0.3906474765984251, -1.6795440797869192}, 
      frc::Vectord<2>{0.4555656397444899, -1.6062496966780575}, 
      frc::Vectord<2>{0.5337250399202257, -1.5161407222231158}, 
      frc::Vectord<2>{0.6238186687888023, -1.4113728720549832}, 
      frc::Vectord<2>{0.724291434560601, -1.2941018628683976}, 
      frc::Vectord<2>{0.8334619726527297, -1.1664834116006377}, 
      frc::Vectord<2>{0.94957424350127, -1.0306732352877277}, 
      frc::Vectord<2>{1.0708246072967331, -0.8888270510165952}, 
      frc::Vectord<2>{1.1953779024599802, -0.7431005759032906}, 
      frc::Vectord<2>{1.3213776873076988, -0.5956495270807209}, 
      frc::Vectord<2>{1.4469533275121016, -0.44862962169072257}, 
      frc::Vectord<2>{1.5702254952340062, -0.3041965768785042}, 
      frc::Vectord<2>{1.689310928609846, -0.16450610978844316}, 
      frc::Vectord<2>{1.8023271394196176, -0.03171393756061583}, 
      frc::Vectord<2>{1.907397655728144, 0.09202422267263387}, 
      frc::Vectord<2>{2.002658344740979, 0.20455265379081963}, 
      frc::Vectord<2>{2.0862656551352696, 0.30371563869123014}, 
      frc::Vectord<2>{2.156408211012723, 0.3873574602987031}, 
      frc::Vectord<2>{2.211325146961678, 0.45332240158355885}, 
      frc::Vectord<2>{2.249339193605777, 0.4994547456003498}, 
      frc::Vectord<2>{2.2689280275926285, 0.5235987755982988}
    });

    return traj;
  }
private:
  ArmTrajectoryParams params;
  units::second_t totalTime{0.0};
  std::vector<frc::Vectord<2>> points;
};