#include "../common/SLIinterfaceAPI.h"
#include "technology/include/SEP/SEP_InstIdx.h"
#include "../common/SLIinterfaceMemory.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/memory/lib/EyeQ/newMemMap.h"
#include "functionality/partialRun/partialRun_API.h"
#include "technology/DS/SLIutils/SLIFoeEstimation_API.h"
#include "technology/brain2/brain2_API.h"
#include "functionality/interface/modelIF.h"
#include "technology/failSafes/interface/failSafesIF.h"
#include "technology/DS/SLIinterface_ServicesUser.h"


using namespace Fix::MEimage;
using namespace std;
extern "C" {
  
void INIT_SLIinitMemory(int instIdx)
{
  SLIinterfaceAPI::initMemory();
}

void SEP_updateMEMemoryCollection(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  SLIinterfaceAPI::updateMEMemoryCollection();
  auto * mmhApi=TSR::getSLIUser().get<MEMem_API::MEMemHarvesterService_API>();
  assert(mmhApi!=nullptr);
  if (mmhApi->isREMEnabled()) {
    // Code was moved to SEP_MEMemSetTSRData. Not sure if need to run anything here if no REM active.
    const Float::MEmath::Mat<4, 4, double> *currVehRT = mmhApi->getCurrVehRT();
    if(NULL != currVehRT){
      float vehicleS = mmhApi->getVehicleS();
      float scale = mmhApi->getSpeedFactor();
      SLIinterfaceAPI::advanceBestEstimation(*currVehRT, vehicleS, scale);
    }
  }
  SLIinterfaceAPI::updateMEMemoryFinalEstimations();
}


void SEP_updateFOE(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  CHECK_FORWARD_CAM;
  SLIinterfaceAPI::updateFOE();
}

void SEP_resetFOE(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  CHECK_FORWARD_CAM;
  const Log::ModelIF& outputModel = Brain2API::getOutputModel();
  const FailSafesIF* failSafes = (outputModel.failSafes != NULL) && outputModel.failSafes->available() ? &(*(*outputModel.failSafes)) : NULL;

  if (failSafes != NULL && failSafes->TSROOOCFailsafeOutput()._foeEstimationResetRequired) {
	  FoeEstimationAPI::reset();
  }
}

void SEP_killingDead(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  CHECK_FORWARD_CAM;
  SLIinterfaceAPI::killingDead();
}


void SEP_dummySLIGoal(int instIdx){}
  
}
