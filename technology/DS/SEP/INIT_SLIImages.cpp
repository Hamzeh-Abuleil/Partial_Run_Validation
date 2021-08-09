#include "technology/DS/common/SLIinterfaceAPI.h"
#include "technology/DS/common/SLIinterfaceMemory.h"
#include "technology/DS/common/SLIProperties.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/brain2/brain2_API.h"
#include "functionality/calibration/clipProperties.h"
#include "technology/DS/SLIutils/common/SLIUtils.h"
#include "basicTypes/image/image.h"
#include "utilities/preProcess/supportMap/supportMap_API.h"
#include "technology/DS/VisionOnly/VisionOnly_API.h"
#include "technology/DS/VisionOnly/hmi_API.h"

//SECONDARY = color
//GENERAL = texture

DECLARE_PREPSYS_VRT_SUP_PYRAMID_SLI()
DECLARE_PREPSYS_HRZ_SUP_PYRAMID_SLI()

namespace Memory{
  SoCptr RED_CLEAR_HISTOGRAM_EU = 0;
}

extern "C" void INIT_SLIImages(int instIdx)
{
  if (Brain2API::getBrain2Properties()->AVdemoMode() ||
      (CameraInfo::exists(CameraInfo::e_FRONT_CORNER_RIGHT) && CameraInfo::exists(CameraInfo::e_REAR_CORNER_RIGHT)) ||
      (CameraInfo::exists(CameraInfo::e_FRONT_CORNER_LEFT) && CameraInfo::exists(CameraInfo::e_REAR_CORNER_LEFT))){
    TSR::getProperties().tsrCameraIndependentMode(true);  // TODO: Change logic so command line property can overwrite this.
  }
  TSR::initDS();
  bool disableTSR = TSR::getProperties().disableTSR() || TSR::getProperties().noSLI();
  TSR::getProperties().disableTSR(disableTSR);
  SLIinterfaceAPI::disableTSR(disableTSR);

  if (CameraInfo::exists(CameraInfo::e_FORWARD)) {
    bool activeVisionOnly = TSR::getProperties().visionOnly() && TSR::exists(TSR::REGULAR_IMAGE);
    VisionOnlyAPI::initVisionOnlyModule(activeVisionOnly, TSR::getImage(0,TSR::SecondaryExp, TSR::REGULAR_IMAGE)->imageRect(),
                                        TSR::getImage(-1,TSR::SecondaryExp, TSR::REGULAR_IMAGE)->imageRect());
    HMI_API::initHMI();
  }
}

