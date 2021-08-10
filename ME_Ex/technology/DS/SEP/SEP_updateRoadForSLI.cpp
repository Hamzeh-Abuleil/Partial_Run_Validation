#include "../../DS/common/SLIinterfaceAPI.h"
#include "technology/road/interface/roadIF.h"
#include "technology/brain2/brain2_API.h"
#include "technology/objectSensing/objectDetection/VD2D/VehiclesService_API.h"
#include "technology/DS/common/SLIinterfaceAPI.h"
#include "technology/DS/VisionOnly/common/VisionOnly.h"
#include "technology/DS/roadMarkings/SLIRoadMarkings_API.h"
#include "technology/DS/SLIutils/common/SLIUtils.h"
#include "technology/worldModel/egoKalman/egoKalman_API.h"
#include "technology/road/core/common/roadMemory.h"
#include "technology/road/core/common/kalmanFilter/egoKalmanFilter_API.h"
#include "technology/DS/SLIinterface_ServicesUser.h"
#include "functionality/partialRun/partialRun_API.h"


extern "C" void SEP_setRoadAndVclIFPointersForSLI(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  CHECK_FORWARD_CAM;
  if (!TSR::isTechActive()) {
    return;
  }
  auto * rApi=TSR::getSLIUser().get<Road_API::RoadService_API>();
  assert(rApi!=nullptr);
  const RoadIF* roadIF = (const RoadIF*)&(**rApi->getRoadSlowIF());
  auto *vApi=TSR::getSLIUser().get<Vehicles_API::VehiclesService_API>();
  assert(vApi!=nullptr);
  const Fix::MEimage::Sync<Log::VclCollectionIF>* vclCollectionPtr = vApi->getVCLCollectionIF();
  if (vclCollectionPtr==NULL)
    return;

  const Log::VclCollectionIF* vclIFvec = &(**vclCollectionPtr);

  if (roadIF == NULL || vclIFvec == NULL)
    return;

  SLIinterfaceAPI::setRoadAndVclIFPointers(roadIF, vclIFvec);
}

extern "C" void SEP_setRelevanceData(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  CHECK_FORWARD_CAM;
  if (!TSR::isTechActive()) {
    return;
  }
  auto *vApi=TSR::getSLIUser().get<Vehicles_API::VehiclesService_API>();
  assert(vApi!=nullptr);
  const Fix::MEimage::Sync<Log::VclCollectionIF>* vclCollectionPtr =vApi->getVCLCollectionIF();
  if (vclCollectionPtr==NULL){
    return;
  }

  const Log::VclCollectionIF* vclIFvec = &(**vclCollectionPtr);
  if (vclIFvec == NULL){
    return;
  }

  VisionOnlyAPI::relevanceData data;

  data.bp_isNear = false;
  data.bp_isNearValue = 0.0f;
  data.bp_Y = 0.0f;
  // data.egoYawRate = EgoKalman::get_egoYawRate();
  // data.egoSpeed = EgoKalman::get_egoSpeed();
  // data.egoYawAccel = EgoKalman::get_egoYawAccel();
  // data.acc = EgoKalman::get_egoAccel();
  data.egoYawRate = WorldModel::wmEgoKalman::getEgoYawRate();
  data.egoSpeed = WorldModel::wmEgoKalman::getEgoSpeed();
  data.egoYawAccel = WorldModel::wmEgoKalman::getEgoYawAccel();
  data.acc = WorldModel::wmEgoKalman::getEgoAccel();

  for (unsigned int i=0 ; i < vclIFvec->obstaclesNum ; i++){
    const Log::VclIF& vclIF = vclIFvec->obstacles[i];
    if (vclIF.cipv()) {
      data.hasCipv = true;
      data.cipvRelativeSpeed = vclIF.relativeSpeedVisionOnly();
      data.cipvRelativeDistance = vclIF.distanceVisionOnly();
    }
  }

  data.closestStopLineDistance = 1000.f;

  const MEtypes::ptr_vector<RoadMarkings_StopLine>* stopLineVec = &(RoadMarkings::API::getRoadMarkingsApprovedStopLines());
  for (size_t k = 0; k < (*stopLineVec).size(); k++){
    if(((*stopLineVec)[k]).realWorldZ < data.closestStopLineDistance){
      data.closestStopLineDistance = ((*stopLineVec)[k]).realWorldZ;
      if(data.closestStopLineDistance < 40.f){
        data.existingStopLine = true;
      }
    }
  }

  auto * rApi=TSR::getSLIUser().get<Road_API::RoadService_API>();
  assert(rApi!=nullptr);
  if(rApi->roadT0FastDataForSlow() != NULL){
    const RoadCeNetModelIF &ceNetModel = rApi->roadT0FastDataForSlow()->get_ceNetModel();

    data.ceNet_da = ceNetModel.model[0];
    data.ceNet_a = ceNetModel.model[1];
    data.ceNet_b = ceNetModel.model[2];
    data.ceNet_c = ceNetModel.model[3];
    data.ceNet_valid = ceNetModel.valid;
    data.ceNet_startZ = ceNetModel.viewRange.startZ;
    data.ceNet_endZ = ceNetModel.viewRange.endZ;
  }

  VisionOnlyAPI::setRelevanceData(data);
}

extern "C" void SEP_runSLCRelevanceModel(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);
  CHECK_FORWARD_CAM;
  if (!TSR::isTechActive()) {
    return;
  }
  VisionOnlyAPI::runSLCRelevanceModel();
}

