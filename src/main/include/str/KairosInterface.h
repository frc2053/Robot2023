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
  };

  void SetConfig(std::string jsonConfig) {
    configPublisher.Set(jsonConfig);
  };

  void Request(ArmTrajectoryParams params) {
    
  };

private:
  nt::StringPublisher configPublisher;
  nt::StringPublisher requestPublisher;
  nt::DoubleArraySubscriber resultSubscriber;
};
