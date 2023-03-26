#pragma once

#include <networktables/DoubleArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/IntegerTopic.h>
#include <str/ArmTrajectory.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <inttypes.h>

struct KairosResults {
  std::vector<double> shoulderPoints{};
  std::vector<double> elbowPoints{};
  units::second_t totalTime{0};
  std::size_t hash{0};
};

class KairosInterface {
public:
  KairosInterface() {
    nt::PubSubOptions fastUpdateOptions;
    fastUpdateOptions.periodic = 0.0;

    nt::PubSubOptions requestOptions;
    requestOptions.periodic = 0.0;
    requestOptions.keepDuplicates = true;

    std::shared_ptr<nt::NetworkTable> kairosTable = nt::NetworkTableInstance::GetDefault().GetTable("kairos");
    configPublisher = kairosTable->GetStringTopic("config").Publish(fastUpdateOptions);
    requestPublisher = kairosTable->GetStringTopic("request").Publish(requestOptions);

    resultSubscriber = kairosTable->GetDoubleArrayTopic("result/0").Subscribe({}, fastUpdateOptions);
    hashSubscriber = kairosTable->GetStringTopic("result/hash/0").Subscribe({}, fastUpdateOptions);

    kairosTable->PutNumberArray("result/0", {});
    
    LoadTrajectoriesIntoCache();
  };

  void LoadTrajectoriesIntoCache() {
    std::string filePath = frc::filesystem::GetDeployDirectory() + "/" + "arm_trajectory_cache.json";
    std::error_code errorCode;
    wpi::raw_fd_istream file(filePath, errorCode);

    if(errorCode) {
      frc::DataLogManager::Log(fmt::format("Error opening arm_trajectory_cache! {}: {}", filePath, errorCode.message()));
    }
    else {
      wpi::json data = wpi::json::parse(file);
      for(const auto& trajectory : data["trajectories"]) {
        ArmTrajectoryParams params;
        params.initialState = frc::Vectord<2>{trajectory["initialJointPositions"][0].get<double>(), trajectory["initialJointPositions"][1].get<double>()};
        params.finalState = frc::Vectord<2>{trajectory["finalJointPositions"][0].get<double>(), trajectory["finalJointPositions"][1].get<double>()};
        std::size_t currentHash = std::hash<ArmTrajectoryParams>{}(params);
        KairosResults results;
        results.hash = currentHash;
        results.totalTime = units::second_t{trajectory["totalTime"].get<double>()};
        for(std::size_t i = 0; i < (trajectory["points"].size() - 1) / 2; i++) {
          results.shoulderPoints.push_back(trajectory["points"][i * 2].get<double>());
          results.elbowPoints.push_back(trajectory["points"][i * 2 + 1].get<double>());
        }
        kairosCache[currentHash] = results;
      }
    }
  };

  void Update() {
    for(const nt::ConnectionInfo& con : nt::NetworkTableInstance::GetDefault().GetConnections()) {
      if(!isConnected && con.remote_ip == "10.20.53.11") {
        isConnected = true;
      }
    }
    frc::SmartDashboard::PutBoolean("Superstructure/Kairos Connected", isConnected);

    if(resultRecieved) {
      return;
    }

    std::vector<double> results = resultSubscriber.Get();
    std::stringstream ss(hashSubscriber.Get());
    std::size_t hashResult = 0;
    ss >> hashResult;
    if(results.size() > 0) {
      if(hashResult == parameterHash) {
        resultRecieved = true;
        recentResults.hash = hashResult;
        recentResults.totalTime = units::second_t{results[0]};
        recentResults.shoulderPoints.clear();
        recentResults.elbowPoints.clear();
        for(std::size_t i = 0; i < (results.size() - 2) / 2; i++) {
          recentResults.shoulderPoints.push_back(results[i * 2 + 1]);
          recentResults.elbowPoints.push_back(results[i * 2 + 2]);
        }
        kairosCache[hashResult] = recentResults;
      }
    }
  }

  KairosResults GetMostRecentResult() {
    return recentResults;
  }

  void SetConfig(std::string jsonConfig) {
    configPublisher.Set(jsonConfig);
  };

  void Request(ArmTrajectoryParams params) {
    std::size_t currentHash = std::hash<ArmTrajectoryParams>{}(params);

    //break out if we have a cached trajectory
    if(kairosCache.contains(currentHash)) {
      frc::DataLogManager::Log(fmt::format("Arm trajectory is in cache! Setting trajectory and not requesting..."));
      recentResults = kairosCache[currentHash];
      return;
    }

    if(currentHash == parameterHash) {
      frc::DataLogManager::Log(fmt::format("Arm trajectory hashes are equal! We are not going to send another request!"));
      return;
    }

    wpi::json dataToSend;
    dataToSend["hash"] = currentHash;
    dataToSend["initial"] = {params.initialState(0), params.initialState(1)};
    dataToSend["final"] = {params.finalState(0), params.finalState(1)};
    dataToSend["constraintOverrides"] = wpi::json::array();

    frc::DataLogManager::Log(fmt::format("Sending trajectory with hash {} to kairos", currentHash));

    parameterHash = currentHash;
    resultRecieved = false;
    requestPublisher.Set(dataToSend.dump());
  };

private:
  nt::StringPublisher configPublisher;
  nt::StringPublisher requestPublisher;
  nt::DoubleArraySubscriber resultSubscriber;
  nt::StringSubscriber hashSubscriber;
  bool isConnected{false};
  bool resultRecieved{true};
  std::size_t parameterHash{0};
  KairosResults recentResults;
  std::unordered_map<std::size_t, KairosResults> kairosCache;
};
