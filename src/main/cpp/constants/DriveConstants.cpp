#include "constants/DiffDriveConstants.h"
#include <frc/system/plant/LinearSystemId.h>

namespace str {
  namespace diff_drive_consts {
    const frc::LinearSystem<2, 2, 2> DRIVE_TRAIN_PLANT = frc::LinearSystemId::IdentifyDrivetrainSystem(KV, KA, KV_ANGULAR, KA_ANGULAR);
  }   
}   