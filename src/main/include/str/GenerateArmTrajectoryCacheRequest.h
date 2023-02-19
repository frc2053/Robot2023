#pragma once

#include <str/ArmPose.h>
#include <str/ArmConfig.h>
#include <str/TwoJointArmDynamics.h>
#include <constants/ArmConstants.h>
#include <filesystem>
#include <fstream>

bool GenerateArmTrajectoryCacheFile() {
  ArmConfig config{ArmConfig::LoadJson("arm_config.json")};
  frc::Vectord<6> initialState{0.248886761314,-1.8326,0,0,0,0};
  TwoJointArmDynamics armSystem {
    config.shoulder.mass,
    config.elbow.mass,
    config.shoulder.length,
    config.elbow.length,
    config.shoulder.moi,
    config.elbow.moi,
    config.shoulder.cgRadius,
    config.elbow.cgRadius,
    config.shoulder.motor,
    config.elbow.motor,
    config.shoulder.gearReduction,
    config.elbow.gearReduction,
    initialState,
    str::arm_constants::positionTolerance,
    str::arm_constants::velocityTolerance,
    str::arm_constants::controlEffort,
    str::arm_constants::qPos,
    str::arm_constants::qVel,
    str::arm_constants::est,
    str::arm_constants::rPos
  };

  wpi::json cacheRequestFile;
  cacheRequestFile["trajectories"] = wpi::json::array();

  //iterate through all combos of poses
  for(const ArmPose& startPose : AllPoses) {
    for(const ArmPose& endPose : AllPoses) {
      if(startPose.endEffectorPosition != endPose.endEffectorPosition) {
        
        frc::Vectord<2> initialJointAngles = startPose.AsJointAngles(armSystem);
        frc::Vectord<2> endJointAngles = endPose.AsJointAngles(armSystem);

        wpi::json trajectoryJson;
        trajectoryJson["initialJointPositions"].push_back(initialJointAngles(0));
        trajectoryJson["initialJointPositions"].push_back(initialJointAngles(1));
        trajectoryJson["finalJointPositions"].push_back(endJointAngles(0));
        trajectoryJson["finalJointPositions"].push_back(endJointAngles(1));

        cacheRequestFile["trajectories"].push_back(trajectoryJson);

        std::string filePath = std::filesystem::temp_directory_path().string() + "/arm_trajectory_cache_request.json";
        std::ofstream trajectoryRequestFile(filePath);
        trajectoryRequestFile << cacheRequestFile.dump() << std::endl;
        fmt::print("Wrote cache request file to: {}\n", filePath);
      }
    }
  }

  return true;
}