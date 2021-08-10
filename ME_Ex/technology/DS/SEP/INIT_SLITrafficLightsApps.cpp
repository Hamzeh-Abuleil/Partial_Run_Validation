#include "technology/DS/TSRMultiFrame/TrafficLights/TFLJunctionManager/TFLJunctionManager_API.h"
#include "technology/DS/TSRMultiFrame/TrafficLights/Applications/RedTFLWarning/RedTFLWarning_API.h"
#include "technology/DS/dsutils/dsUtils.h"

extern "C" void INIT_SLITrafficLightsApps(int instIdx)
{
  if(TSR::isTrafficLightsActive()){
    TSR::TFLJunctionManager_API::initializeModule();
    TSR::RedTFLWarning_API::initializeModule();
  }
}
