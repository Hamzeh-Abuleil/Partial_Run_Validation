#include "technology/AV/AV_API_internal.h"
#include "technology/DS/common/SLIinterfaceAPI.h"
#include "technology/DS/common/SLIinterfaceMemory.h"
#include "technology/DS/TSRMultiFrame/TrafficLights/TflCameraUtils.h"
#include "technology/DS/dsutils/dsGateway.h"


extern "C" void INIT_SLITrafficLights(int instIdx)
{
  const bool enableTrafficLights = TSR::getProperties().enableTrafficLights() && TSR::exists(TrafficLights::TFLUtils::getPrimaryCamera());
  TSR::getProperties().enableTrafficLights(enableTrafficLights);
  
  const bool demoTfl = TSR::getProperties().demoTfl() || AV_API::AVPropertiesEnabled();
  TSR::getProperties().demoTfl(demoTfl);
  
  const bool is8MP120Camera = TSR::is8MP120Camera();
  const bool isDemo = TSR::getProperties().demoTfl();
  const bool enableTflAlternatingSystems = TSR::getProperties().enableTflAlternatingSystems() || is8MP120Camera || isDemo;
  TSR::getProperties().enableTflAlternatingSystems(enableTflAlternatingSystems);
}
