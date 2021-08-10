#include "technology/DS/TSRMultiFrame/tsrMF_API.h"
#include "technology/DS/TSRMultiFrame/TrafficLights/TFLJunctionManager/TFLJunctionManager_API.h"
#include "technology/DS/TSRMultiFrame/TrafficLights/Applications/RedTFLWarning/RedTFLWarning_API.h"

extern "C" void SEP_runTrafficLightsApps(int instIdx){

  //tfl RedTFLWarning app
  if(TSR::API::isActiveManager(TSR::E_TFL_STRUCTURE_SYSTEM) || TSR::API::isActiveManager(TSR::E_TFL_SPOT_SYSTEM)){
    //TFLJunctionManager_API::registerTFLs(sliIF);
    TSR::TFLJunctionManager_API::execute();
    TSR::RedTFLWarning_API::execute();

    Fix::MEimage::Sync<SLI_IF>* _sliIF = SLIinterfaceAPI::getSLI_IF();
    SLI_IF* sliIF = &(_sliIF->editable());
    TSR::RedTFLWarning_API::fillSLIIF(sliIF);
  }

}
