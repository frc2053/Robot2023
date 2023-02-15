#pragma once

#include <networktables/DoubleArrayTopic.h>
#include <networktables/StringTopic.h>
#include <str/ArmTrajectory.h>

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

    kairosTable->PutNumberArray("result/0", {});
  };

  void SetConfig(std::string jsonConfig) {
    configPublisher.Set(jsonConfig);
  };

  void Request(ArmTrajectoryParams params) {
    wpi::json dataToSend;
    dataToSend["hash"] = std::hash<ArmTrajectoryParams>()(params);
    dataToSend["initial"] = {params.initialState(0), params.initialState(1)};
    dataToSend["final"] = {params.finalState(0), params.finalState(1)};
    dataToSend["constraintOverrides"] = wpi::json::array();

    requestPublisher.Set(dataToSend.dump());
    resultRecieved = false;
  };

private:
  nt::StringPublisher configPublisher;
  nt::StringPublisher requestPublisher;
  nt::DoubleArraySubscriber resultSubscriber;
  bool resultRecieved{false};
};
