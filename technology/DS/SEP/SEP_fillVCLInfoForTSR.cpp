#include "functionality/interface/modelIF.h"
#include "technology/mobilib/fix/common/MEXimage/typeImages.h"
#include "technology/objectSensing/objectDetection/VD2D/VehiclesService_API.h"
//#include "technology/DS/ConstructionArea/common/ConstructionAreaManager.h"
#include "technology/DS/roadMarkings/SLIRoadMarkings_API.h"
#include "technology/DS/SLIutils/common/SLIUtils.h"
#include "technology/DS/common/SLIinterfaceAPI.h"
#include "technology/DS/TSRMultiFrame/i386/tsrMFDebuggerUtils.h"
#include "technology/objectSensing/utilities/cameras/vcls_cameras.h"
#include "functionality/partialRun/partialRun_API.h"

#define ROAD_MARKINGS_MAX_VCL_CIPV_DISTANCE 20


extern "C" void SEP_TSRFillVD3DInfoForTSR(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  if (TSR::getProperties().disableTSR() ||
      TSR::getActiveImageKeys().empty()) {
      return;
    }
  static const bool sSLIOnly = Debug::Args::instance().existsParameter("-sSLIOnly");
  if (!sSLIOnly) {
    TSR::updateVd3dCandidates();
  }
}

extern "C" void SEP_fillVCLInfoForTSR(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  if (!SLIinterfaceAPI::checkSliActivation()) {
    return;
  }
  
  static const bool sSLIOnly = Debug::Args::instance().existsParameter("-sSLIOnly");
  if (!sSLIOnly) {
    TSR::updateVclCandidates();
  }
}

