#pragma once

#include <frc/PowerDistribution.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/PowerDistributionSim.h>
#include <frc/simulation/BatterySim.h>
#include <units/current.h>

namespace str {
  class PDP {
  public:
    static PDP& GetInstance() {
      static PDP instance;
      return instance;
    };
    PDP(PDP const&) = delete;
    void operator=(PDP const&) = delete;

    frc::PowerDistribution* GetPDP() {
      return &pdp;
    };

    void SetInputVoltage(units::volt_t voltage) { sim.SetVoltage(voltage.to<double>()); };
    void SetCurrentOnChannel(int channel, units::ampere_t amps) { sim.SetCurrent(channel, amps.to<double>()); };
    units::volt_t GetBatteryVoltageWithLoad() { return frc::sim::BatterySim::Calculate({{GetTotalCurrentDraw()}}); };
  private:
    units::ampere_t GetTotalCurrentDraw() {
      units::ampere_t totalAmps = 0_A;
      for(int i = 0; i < 21; i++) {
        totalAmps = totalAmps + units::ampere_t{pdp.GetCurrent(i)};
      }
      return totalAmps;
    }
    PDP() {
    };
    frc::PowerDistribution pdp;
    frc::sim::PowerDistributionSim sim{pdp};
  };
}   