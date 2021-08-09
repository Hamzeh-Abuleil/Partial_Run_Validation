#include "technology/include/SEP/SEP_InstIdx.h"
#include "technology/pedestrians/pedestrians/common/pedEnums.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/matching/radarMatching_API.h"
#include "technology/brain2/brain2_API.h"
#include "technology/pedestrians/pedestrians/common/Pedestrians_API_Internal.h"
#include "technology/pedestrians/pedestrians/common/commonResources.h"
#include "technology/pedestrians/pedestrians/pedSingleFrame/common/pedSingleFrameManager_API_internal.h"
#include "technology/pedestrians/pedestrians/cliquer/common/cliquer_API_internal.h"
#include "technology/pedestrians/pedestrians/residualMotion/residualMotion_API.h"
#include "technology/pedestrians/pedestrians/common/speedAndDistance.h"
#include "technology/pedestrians/pedestrians/common/bicycleWheelsDetectionSF.h"
#include "technology/pedestrians/pedestrians/pedTracking/pedTracking_API.h"
#include "technology/pedestrians/pedestrians/pedStereo/pedStereo_API.h"
#include "functionality/interface/applicationData.h"
#include "technology/FCW/pedFCWProcess/pedFCWProcess_internal_API.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "technology/brain2/brainDefaults.h"
#include "technology/brain2/common/brain2Properties.h"
#include "technology/FCW/common/AEB_HMI_API.h"
#include "technology/FCW/FCW_API.h"
#include "technology/FCW/PedsFCWTester/RunFlowReader.h"
#include "basicTypes/mfl/common/itrkWriter_API_internal.h"
#include "basicTypes/mfl/itrkWriter_API.h"
#include "technology/FCW/fcwValidation/ObjFOE_API.h"
#include "technology/mobilib/fix/common/MEXmisc/timeStampUtils.h"
#include "utilities/vehicleInformation/vehicleInformation_API.h"
#include "technology/pedestrians/pedestrians/sceneAnalysis/freeSpaceMFAnalyzer.h"
#include "technology/pedestrians/lib/PLM/PLM_API.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "utilities/nn/neuralNetwork_API.h"
#include "functionality/partialRun/partialRun_API.h"
//Crossing Vehicles New Design
#include "technology/FCW/CrossingVeh/API/CVFCW_internal_API.h"

#include "functionality/partialRun/wrappers/PedCollectionIF_wrapper.h"
#include "functionality/partialRun/wrappers/PedFCWIF_wrapper.h"
#include "functionality/partialRun/wrappers/PedCliquesInfo_wrapper.h"

#include "technology/FCW/pedFCWProcess/pedFCWProcess_API.h"
#include "utilities/imageInformation/generalImageInfo.h"
extern int globalFrameIndex;
extern int globalBrainIndex;

//#define PRINT_SET_ENTRIES

PrepSys::exp_mask getExposureBySEPInstance(int inst){
  if (inst==SEP::Exp_T1){
    return PrepSys::exp_mask::IMG1;
  }
  else if (inst==SEP::Exp_T2){
    return PrepSys::exp_mask::IMG3;
  }
  else {
    assert(0);
    return  PrepSys::exp_mask::IMG1;
  }
}

//void readFromLogProcess();

using namespace  Pedestrians;

extern "C" void SEP_fillPedResultForCrossing(int instIdx)
{
RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians_API::fillPedResultForCrossing(instIdx);
}

extern "C" void SEP_pedInitSingleFrame(int instIdx)
{
  // For dependencies purpose
}
gsf_task_def ped_sfda_process_task;
gsf_task_def ped_sfda_attention_task;

void pedInitSingleFrameGSF(PrepSys::exp_mask exposure, CameraInfo::CameraInstance cinst ) {
  CameraInfo::CameraInstance rectCamInst = ((CameraInfo::CameraInstance )cinst == CameraInfo::CameraInstance::e_FORWARD_FISHEYE)?CameraInfo::CameraInstance::e_FORWARD_FISHEYE_RECTIFIED:(CameraInfo::CameraInstance )cinst;
  
  if(CommonResources::instance().doesCameraExist(CameraInfo::e_FORWARD_RECTIFIED)){
     if(rectCamInst == CameraInfo::e_FORWARD){
        rectCamInst = CameraInfo::e_FORWARD_RECTIFIED;
     }
  }
  if (CommonResources::instance().isClassificationInstance(exposure,rectCamInst)){
   GSF_START(&ped_sfda_process_task, {
       Pedestrians_API::pedInitSingleFrame(exposure,rectCamInst);
    });
  }
  else{
  // Create dummy goals for cases that we run freespace on less images than what is defined in the agenda
   GSF_START(&ped_sfda_process_task, {});
 }
}

extern "C" void  SEP_pedInitSingleFrameT0(int camInst)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  const std::vector<MEImageInfo::ImageInfoId::ImageKey>& camInstVec = CommonResources::instance().getImageKeys();
  
  int numberOfCameras = camInstVec.size();
  for (int camIndex = 0; camIndex < numberOfCameras ; ++camIndex) {
    pedInitSingleFrameGSF(PrepSys::exp_mask::IMG1, CommonResources::instance().getCamInstByImageKey(camInstVec[camIndex]));
  }
}
extern "C" void  SEP_pedInitSingleFrameT1(int camInst)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  const std::vector<MEImageInfo::ImageInfoId::ImageKey>& camInstVec = CommonResources::instance().getImageKeys();
  int numberOfCameras = camInstVec.size();
  for (int camIndex = 0; camIndex < numberOfCameras ; ++camIndex) {
    pedInitSingleFrameGSF(PrepSys::exp_mask::IMG3,CommonResources::instance().getCamInstByImageKey(camInstVec[camIndex]));
  }
}
extern "C"  void SEP_saveT2TimeStamp(int instIdx)
{
RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  Pedestrians_API::saveT2TimeStamp();
}
#include "utilities/cameraInformation/cameraInformation_API.h"
extern "C" void SEP_filterCliquesFillCrossingInformation(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::filterCliquesFillCrossingInformation(instIdx);
}

extern "C" void SEP_storeBikesCliquesData(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::storeBikesCliquesData(instIdx);
}
//---------------------- fft -------------------------------------
/* INIT_FFTInitVMP
 * =====================
 * module VMP initialization
 */

int getLevelFromInst(int instIdx) {

  switch (instIdx) {
  case SEP::Lm2:
    return -2;    
  case SEP::Lm1:
    return -1;    
  case SEP::L0:
    return 0;  
  case SEP::L1:
    return 1;   
  case SEP::L2:
    return 2;    
  default:
    assert(false);
    return 0;
  }
}


extern "C" void SEP_fillSpotsArray(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_NV_PEDS_DISABLED; //never shouldn't be true - this goal isn't suppost to run on NV peds
  Pedestrians::fillSpotsArray();
}

// update for each approve clique if its lane
extern "C" void SEP_updateLaneAssignement(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::updateLaneAssignement(instIdx);
}

//---------------- residual motion -------------------------------

extern "C" void INIT_initResidualMotionManager(int instIdx)
{
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  ResidualMotion::initResidualMotionManager();
}

extern "C" void SEP_filterCliquesClassifier(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::filterCliquesClassifier(instIdx);
}

extern "C" void SEP_filterCliquesClassifyResidualMotionUpdateFrame(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::filterCliquesClassifyResidualMotionUpdateFrame(instIdx);
}


extern "C" void SEP_filterCliquesClassifyResidualMotionGetResult(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::filterCliquesClassifyResidualMotionGetResult(instIdx);
}

extern "C" void SEP_filterCliquesClassifyMotionSegmentation(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  if (CommonResources::instance().isWideSetup()) {
    return;
  }
  Pedestrians::filterCliquesClassifyMotionSegmentation(instIdx);
}

//------------------ stereo ----------------------------
extern "C" void SEP_pedStereoAssist(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::pedStereoAssist();
}

//----------------- crossing filter ---------------------

extern "C" void SEP_filterCliquesPrepareCrossingFilter(int instIdx)
{
//#ifdef PEDESTRIANS_ROOT_WONO_LIB
//   return;
//#endif
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_PEDS_IF_NO_T2(instIdx);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::filterCliquesPrepareCrossingFilter(instIdx);
}

extern "C" void SEP_filterCliquesPostCrossingFilter(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::filterCliquesPostCrossingFilter(instIdx);
}


extern "C" void SEP_filterCliquesFinalFilters(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_PEDS_IF_NO_T2(instIdx);
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  if (SEP::Exp_T2 == instIdx) { // we do it here, cause we don't call for the crossing function like in T1
    SpeedAndDistance::instance().calcOnAllCliques(false/*endOfFrame*/, SEP::Exp_T2);
  }
  else if (SEP::Exp_T1 == instIdx && !CommonResources::instance().isFCWSetup()){
    SpeedAndDistance::instance().calcOnAllCliques(false/*endOfFrame*/, SEP::Exp_T1);
  }
  Pedestrians::filterCliquesFinalFilters(instIdx);
}
extern "C" void SEP_updateSignatures(int instIdx){
    if (SEP::Exp_T1 == instIdx){
        Pedestrians::updateSignatures(instIdx);
    }
}

extern "C" void SEP_copyNonPedObsticles(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::copyNonPedObsticles(instIdx);
}

extern "C" void SEP_pedApproveMFObjectsPart1(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians_API::pedApproveMFObjects_part1(instIdx);
}

extern "C" void SEP_pedApproveMFObjectsPart2(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians_API::pedApproveMFObjects_part2(instIdx);
}

extern "C" void SEP_pedUpdateFSData(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  
  Pedestrians_API::updatePedCliquesFSInfo();
}

extern "C" void SEP_pedFrameEnd(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  Pedestrians_API::pedFrameEnd();
}

extern "C" void SEP_calcOnAllCliquesAtEnd(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  Pedestrians_API::calcOnAllCliquesAtEnd(instIdx);
}

extern "C" void SEP_pedAddPedsToITrackFile(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians_API::pedAddPedsToITrackFile(instIdx);
}


extern "C" void SEP_pedCollectionClear(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians_API::clear();
}

extern "C" void SEP_cliquerAdvanceCliques(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::cliquerAdvanceCliques(instIdx);
  if (instIdx==SEP::Exp_T1){
	  PedFCW::pedFCWObjectsAdvance();
  }
  
}

//-------------- Peds tracking ---------------------

extern "C" void SEP_prepareGridTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  PrepSys::exp_mask  exposure = getExposureBySEPInstance(instIdx);
  Pedestrians::pedTracking_checkCliquesForGridTracking(exposure);
  const std::vector<CameraInfo::CameraInstance>& camInsts = CommonResources::instance().getExposureCameras(exposure);
  int numberOfCameras = camInsts.size();
  for (int camIndex=0; camIndex<numberOfCameras; ++camIndex ){
    Pedestrians::pedTracking_prepareGridTracking(exposure, camInsts[camIndex]);
  }
}
extern "C" void SEP_clqKalmanPredict(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  PrepSys::exp_mask exposure = getExposureBySEPInstance(instIdx);
  CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
  if(exposure != PrepSys::exp_mask::IMG1 && CommonResources::instance().isParkingCamera(defaultCam)){
      return;
  }

  Pedestrians::pedTracking_clqKalmanPredict(exposure);
}
extern "C" void SEP_copyLaneDataFromRoad(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
CHECK_FORWARD_CAM;
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  CommonResources::instance().updateRoadIF();
}

extern "C" void SEP_copyVdPedRelIF(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  CommonResources::instance().copyVdPedRelIF();
}
extern "C" void SEP_predictClqTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  PrepSys::exp_mask exposure = getExposureBySEPInstance(instIdx);
  CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
  if(exposure != PrepSys::exp_mask::IMG1 && CommonResources::instance().isParkingCamera(defaultCam)){
     return;
  }
  Pedestrians::pedTracking_clqPredictClqTracking(exposure);
}

extern "C" void SEP_prepareClqTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  PrepSys::exp_mask exposure = getExposureBySEPInstance(instIdx);
  Pedestrians::pedTracking_prepareClqTracking(exposure);
}

extern "C" void SEP_postClqTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
#ifdef PRINT_SET_ENTRIES
  printf("SEP_postClqTracking. instIdx=%d\n",instIdx);
#endif
  PrepSys::exp_mask exposure = getExposureBySEPInstance(instIdx);
  Pedestrians::pedTracking_postClqTracking(exposure);
}

extern "C" void SEP_postClqBackwardTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::pedTracking_postClqBackwardTracking(instIdx);
}

//-------------- Peds intermediate tracking ---------------------
extern "C" void SEP_applyClqInterTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
#ifdef PRINT_SET_ENTRIES
  printf("SEP_applyClqInterTracking. instIdx=%d\n",instIdx);
#endif
  PrepSys::exp_mask  exposure = getExposureBySEPInstance(instIdx);
  CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
  if(exposure != PrepSys::exp_mask::IMG1 && CommonResources::instance().isParkingCamera(defaultCam)){
      return;
  }
  Pedestrians::pedTracking_applyClqInterTracking(exposure);
}

//-------------- Peds intermediate tracking ---------------------
extern "C" void SEP_postClqInterTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
#ifdef PRINT_SET_ENTRIES
  printf("SEP_postClqInterTracking. instIdx=%d\n",instIdx);
#endif
  PrepSys::exp_mask  exposure = getExposureBySEPInstance(instIdx);
  CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
  if(exposure != PrepSys::exp_mask::IMG1 && CommonResources::instance().isParkingCamera(defaultCam)){
      return;
  }
  Pedestrians::pedTracking_postClqInterTracking(exposure);
}

extern "C" void SEP_postClqBackwardInterTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  PrepSys::exp_mask  exposure = getExposureBySEPInstance(instIdx);
  CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
  if(exposure != PrepSys::exp_mask::IMG1 && CommonResources::instance().isParkingCamera(defaultCam)){
      return;
  }
  Pedestrians::pedTracking_postClqBackwardInterTracking(instIdx);
  //Pedestrians::pedTracking_clqCorrectClqTrackingPrediction(instIdx);
}


extern "C" void SEP_retrackFailed(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::pedTracking_retrackFailed(instIdx);
}

// extern "C" void SEP_correctClqTrackingPrediction(int instIdx)
// {
//   RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
// RETURN_IF_MAIN_PEDS_DISABLED;
//   RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
//   Pedestrians::pedTracking_clqCorrectClqTrackingPrediction(instIdx);
// }

//-------------- Fcw Clique Tracking -----------------

extern "C" void SEP_applyFcwClqTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }

RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::pedTracking_applyFcwClqTracking(instIdx);
}

extern "C" void SEP_applyFcwClqBackwardTracking(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }


RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
#ifdef PRINT_SET_ENTRIES
  printf("SEP_postFcwClqTracking - instIdx=%d\n",instIdx);
#endif
  Pedestrians::pedTracking_postFcwClqTracking(instIdx);
}

//----------------- PedSingleFrame ---------------------
extern "C" void SEP_pedNNAttention(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
}

void pedNNAttentionGSF(PrepSys::exp_mask exposure ,CameraInfo::CameraInstance cinst,bool skipCam){
  
  CameraInfo::CameraInstance rectCamInst = (cinst == CameraInfo::CameraInstance::e_FORWARD_FISHEYE)?CameraInfo::CameraInstance::e_FORWARD_FISHEYE_RECTIFIED:(CameraInfo::CameraInstance )cinst;

  if(CommonResources::instance().doesCameraExist(CameraInfo::e_FORWARD_RECTIFIED)){
     if(rectCamInst == CameraInfo::e_FORWARD){
         rectCamInst = CameraInfo::e_FORWARD_RECTIFIED;
     }
  }
  if (CommonResources::instance().isClassificationInstance(exposure,rectCamInst)){
    GSF_START(&ped_sfda_attention_task, {
        if(!skipCam){
          Pedestrians::pedNNAttention(exposure,rectCamInst);
        }
    });
  }
  else{
    // Create dummy goals for cases that we run freespace on less images than what is defined in the agenda
    GSF_START(&ped_sfda_attention_task, {});
  }
  
}

extern "C" void SEP_pedNNAttentionMCT0(int camInst)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  const std::vector<MEImageInfo::ImageInfoId::ImageKey>& camInstVec = CommonResources::instance().getImageKeys();
  static bool skipFront = false;
  bool skipWide = false;
  bool isSystemOverload = false;
  bool isWono  = CommonResources::instance().isNCAP100Setup();
  bool isWide  = CommonResources::instance().isWideSetup();
  bool is3Fov  = CommonResources::instance().isTrifocalClasterInSetup();
  if(isWono ||isWide){
      isSystemOverload = CommonResources::instance().CommonResources::instance().getNumBigObjects() > 8;
      if(isSystemOverload){
        skipFront = !skipFront;
      }
  }
  else{
      if(is3Fov){
          isSystemOverload = (globalFrameIndex > 0 && CommonResources::instance().getNumBigObjects() > 15);

          if(isSystemOverload){
             if(VehicleInfo::egoSpeed(-1) > 5){
                skipWide = true;
             }
          }
      }
  }


  int numberOfCameras = camInstVec.size();
  for (int camIndex = 0; camIndex < numberOfCameras ; ++camIndex) {
      CameraInfo::CameraInstance camToExam = CommonResources::instance().getCamInstByImageKey(camInstVec[camIndex]);

      if(isSystemOverload &&
         ((skipFront && camToExam == CameraInfo::e_FORWARD)||
          (!skipFront && isWono && camToExam == CameraInfo::e_FORWARD_NCAP100_RECTIFIED )||
          (!skipFront && isWide && camToExam == CameraInfo::e_FORWARD_WIDE_RECTIFIED))){
          pedNNAttentionGSF(PrepSys::exp_mask::IMG1,camToExam,true);
      }
      else{
          if(isSystemOverload &&
                  ((skipWide   && camToExam == CameraInfo::e_FORWARD_FISHEYE)||
                   (!skipWide && is3Fov && camToExam == CameraInfo::e_FORWARD_NARROW ))){
              pedNNAttentionGSF(PrepSys::exp_mask::IMG1,camToExam,true);
          }
          else{
              pedNNAttentionGSF(PrepSys::exp_mask::IMG1,camToExam,false);
          }
      }
  }
}

extern "C" void SEP_pedNNAttentionMCT1(int camInst)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  if(Pedestrians::getNumOfCliquesPrevFrame(SEP::Exp_T1) > 20 && Pedestrians::getNumOfApprovedCliquesPrevFrame(SEP::Exp_T1) > 8){
    return;
  }
  const std::vector<MEImageInfo::ImageInfoId::ImageKey>& camInstVec = CommonResources::instance().getImageKeys();
  int numberOfCameras = camInstVec.size();
  for (int camIndex = 0; camIndex < numberOfCameras ; ++camIndex) {
    pedNNAttentionGSF(PrepSys::exp_mask::IMG3,CommonResources::instance().getCamInstByImageKey(camInstVec[camIndex]),false);
  }
}

extern "C" void SEP_sfAsilValidation(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
}

extern "C" void SEP_uniteT1andT2Cliques(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::uniteT1andT2Cliques();
  Pedestrians::reportToLoadBalance();
  Pedestrians::CliquerSync::updateIdsVector();
  Pedestrians::updatePLMStatus();
}
/*NN*/
extern "C" void SEP_filterCliquesClassifyNN(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_PEDS_IF_NO_T2(instIdx);
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::filterCliquesClassifyNN(instIdx,0);
}

extern "C" void SEP_filterCliquesSetPose(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_PEDS_IF_NO_T2(instIdx);
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::filterCliquesSetPose(instIdx,0);
}


extern "C" void SEP_pedOrBicycleMF(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::updatePedOrBicycleScore(instIdx);
}

extern "C" void SEP_pedUpdateDetectionIF(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::cliquerfillPedDetectionIF(instIdx);
}

extern "C" void SEP_detectHeads_1(int instIdx){
      RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
      RETURN_IF_MAIN_PEDS_DISABLED;
      RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
      RETURN_PEDS_IF_NO_T2(instIdx);
      Pedestrians::detectHeads(instIdx,1);
}

extern "C" void SEP_detectHeads_2(int instIdx){
      RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
      RETURN_IF_MAIN_PEDS_DISABLED;
      RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
      RETURN_PEDS_IF_NO_T2(instIdx);
      Pedestrians::detectHeads(instIdx,2);
}

extern "C" void SEP_pedOrBikeMF(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::cliqueClassifyCliquesType(instIdx);
}


extern "C" void SEP_crossingLegsMF(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::crossingLegsMF(instIdx);
}

/*
extern "C" void SEP_runWaistupAlignmentMF(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  Pedestrians::runWaistupAlignment(instIdx);
}
*/

// lbp classifier
extern "C" void SEP_prepPedLbpMF(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::filterCliquesPrepClassifyLbp(instIdx);
}
extern "C" void SEP_postPedLbpMF(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::filterCliquesPostClassifyLbp(instIdx);
}

////////////////////////

extern "C" void SEP_pedDfovMatching()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  Pedestrians_API::dfovMatching();
}

extern "C" void SEP_prepareBikesWaistups()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  Pedestrians_API::prepareBikesWaistupForCopy();
}
////////////////////////

extern "C" void SEP_pedUpdateAsilSafeStatus(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;

  Pedestrians_API::pedUpdateAsilSafeStatus(SEP::Exp_T1);
}

extern "C" void SEP_pedUpdateModelIF(int instIdx)
{
  RETURN_IF_MAIN_PEDS_DISABLED;

  PartialRun_API::updatePartialRunLoad(CommonResources::instance().getPedCollectionIFWrapper(), PartialRun::IFCategory::PED_IF);

  if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::Pedestrians)){
    SpeedAndDistance::instance().calcKalmanStatisticsOnAllCliques(SEP::Exp_T1);
    //T1 only
    Pedestrians_API::updatePedColIf(SEP::Exp_T1); //TODO: change to get all inst
    if (!BrainDefaults::brainDefaults().dfov || (Brain2API::getBrain2Properties()->isMaster.loaded && !Brain2API::isMaster())){
      Pedestrians_API::fillSlowAgenda();
    }
  }
  Pedestrians_API::fillAVNotApproved();
  PartialRun_API::updatePartialRunStore(CommonResources::instance().getPedCollectionIFWrapper(), PartialRun::IFCategory::PED_IF);
}

/* Cliques Creation*/

extern "C" void SEP_cliquerKillMatched()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
 RETURN_IF_MAIN_PEDS_DISABLED;
 RETURN_PEDS_IF_NO_T2(SEP::Exp_T2);
  Pedestrians::cliquerKillMatched();
}

extern "C" void SEP_cliquerPrepareCreateCliques(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::cliquerPrepareCreateCliques(instIdx);
}


extern "C" void SEP_cliquerCreateCliquesPart1(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  //T1 only
  //Pedestrians::cliquerPrepAlignPed0(SEP::Exp_T1);
}

extern "C" void SEP_cliquerCreateCliquesPart1_T2(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  //T2 only
  //Pedestrians::cliquerPrepAlignPed0(SEP::Exp_T2);
}


extern "C" void SEP_cliquerAlignNNPart1(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
RETURN_PEDS_IF_NO_T2(instIdx);
  if(instIdx != SEP::Exp_T1 && instIdx != SEP::Exp_T2){
    return;
  }
  Pedestrians::cliquerAlignNN(instIdx,0);
}


extern "C" void SEP_cliquerCreateCliquesPart1_N1(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_NV_PEDS_DISABLED;
  //N1 only
  //Pedestrians::cliquerPrepAlignPed0(SEP::Exp_N1);
}


extern "C" void SEP_cliquerCreateCliquesPart2(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  /*Empty*/
}

extern "C" void SEP_cliquerCreateCliquesPart3(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
   /*Empty*/
}

extern "C" void SEP_cliquerCreateCliquesPart4(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  /*Empty*/
}

extern "C" void SEP_cliquerCreateCliquesPart5(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
    /*Empty*/
}

extern "C" void SEP_cliquerCreateCliquesPart6(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::cliquerEliminateNonDengerousCliques(instIdx);
}

extern "C" void SEP_postCrossBicycleCheck(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::cliquerPostCrossBicycleCheck(instIdx);
}

extern "C" void SEP_updateCrossingBicycleDecision(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_PEDS_RUNMODE_IS_NOT_MATCH_TO_INST(instIdx);
  RETURN_PEDS_IF_NO_T2(instIdx);
  Pedestrians::updateCrossingBicycleDecision(instIdx);
}


// Pedestrians Matching and update of PedIF
extern "C" void SEP_pedMatching()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  // Match to radar
  static const bool matchOnly = Debug::Args::instance().existsParameter("-sMatchOnly");
  bool fusionEnable = Radar::e_FUSION == RadarMatching_API::getMode();


  if ((!fusionEnable && !matchOnly) || globalBrainIndex < 1) {
    return;
  }

  MEtypes::ptr_vector<Radar::PedObstacle>& pedestrians = RadarMatching_API::getPedApproved();
  MEtypes::ptr_vector<Radar::PedObstacle>& pedestriansSuspects = RadarMatching_API::getPedSuspects();
  pedestrians.clear();
  pedestriansSuspects.clear();


  Pedestrians_API::getPedRadarMatches(pedestrians, pedestriansSuspects);
  RadarMatching_API::calcPeds(globalBrainIndex, pedestrians, pedestriansSuspects);

  if (!matchOnly){
    Pedestrians_API::setPedRadarMatches (pedestrians, pedestriansSuspects);
  }
}

/*
extern "C" void SEP_pedFcwFrameUpdatePanic(int indx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }


RETURN_IF_MAIN_PEDS_DISABLED;
RETURN_PEDS_IF_NO_T2(indx);
  bool areSet = false;
  dstruct_t sensitivityParams = Brain2API::getSensitivityParams(areSet);

  int pedFCWLevel = -1;
  static int prevPedFCWLevel = -1;

  if (areSet) {
    DKEYTRYLOCAL(&sensitivityParams, ped_level);
    if (DKEYVALID(ped_level)) {
      pedFCWLevel = dsgetb(int32_t, &sensitivityParams, ped_level);
      if (pedFCWLevel != prevPedFCWLevel)
      {
        prevPedFCWLevel = pedFCWLevel;
        PedFCW::adjustSensitivity(pedFCWLevel);
      }
    }
  }
  bool isPanicBrake = PedFCW::pedFCWframeUpdatePanic(indx);
  Pedestrians::cliquerUpdatePostPanicBrakingState(indx,isPanicBrake);
  if(indx == SEP::Exp_T1 || indx == SEP::Exp_T2){
    Pedestrians_API::duplicatePanicClique(indx);
  }
}

void pedFcwFrameUpdate()
{
  bool areSet = false;
  dstruct_t sensitivityParams = Brain2API::getSensitivityParams(areSet);

  int pedFCWLevel = -1;
  static int prevPedFCWLevel = -1;

  if (areSet) {
    DKEYTRYLOCAL(&sensitivityParams, ped_level);
    if (DKEYVALID(ped_level)) {
      pedFCWLevel = dsgetb(int32_t, &sensitivityParams, ped_level);
      if (pedFCWLevel != prevPedFCWLevel)
      {
        prevPedFCWLevel = pedFCWLevel;
        PedFCW::adjustSensitivity(pedFCWLevel);
      }
    }
  }

  dstruct_t* driverProfile = Brain2API::getDriverProfileInput();
  if (NULL != driverProfile && NULL != driverProfile->stype) {
    DKEYTRYLOCAL(driverProfile, sSensitivity);
    if (DKEYVALID(sSensitivity)) {
      dstruct_t dsSensitivity = dsgets(driverProfile, sSensitivity);
      DKEYLOCAL(&dsSensitivity, ipb_pcw_sensitivity_available);
      if (dsgetb(dbool_t, &dsSensitivity, ipb_pcw_sensitivity_available)) {
        DKEYLOCAL(&dsSensitivity, ipb_pcw_sensitivity);
        int gapSensitivity = dsgetb(int32_t, &dsSensitivity, ipb_pcw_sensitivity);
        static int lastGapSensitivity = -1;
        if ( lastGapSensitivity != gapSensitivity ) {
          lastGapSensitivity = gapSensitivity;
          //fprintf(stderr, "pcwGapSensitivity = %d\n", lastGapSensitivity);
          PedFCW::adjustSensitivity(lastGapSensitivity);
        }
      }
    }
  }

  PedFCW::pedFCWframeUpdate();
  PedFCW::logRunFunc("pedFcwFrameUpdate", 0);
}

extern "C" void SEP_fillPedFCWIF()
{
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }

  PedFCW::fillPedFCWIF();
}

extern "C" void SEP_setAlertByValidation(int instIdx) 
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);

RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::setAlertByValidation(instIdx);
  PedFCW::logRunFunc("SEP_setAlertByValidation", instIdx);
}

extern "C" void SEP_pedValidateTTC(int instIdx) {
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_PEDS_IF_NO_T2(instIdx);
RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::validateTTC(instIdx);
  PedFCW::logRunFunc("SEP_pedValidateTTC", instIdx);
}

extern "C" void SEP_pedValidateROI(int instIdx) {
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::validateROI(instIdx);
  PedFCW::logRunFunc("SEP_pedValidateROI", instIdx);
}

extern "C" void SEP_pedValidatePosition(int instIdx) {
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::validatePosition(instIdx);
  PedFCW::logRunFunc("SEP_pedValidatePosition", instIdx);
}

extern "C" void SEP_pedValidateCO(int instIdx) {
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::validateCO(instIdx);
  PedFCW::logRunFunc("SEP_pedValidateCO", instIdx);
}

extern "C" void SEP_pedFcwUpdateSignals() {
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  PedFCW::updateSignalsFor9fps();
}

extern "C" void SEP_pedFcwFrameUpdate ()
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  if (RadarMatching_API::getMode() == Radar::e_FUSION) {
    //pedFcwFrameUpdate();
  }
}

extern "C" void SEP_pedFcwAccelCalc()
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::pedFCWAccelCalc();
}

extern "C" void SEP_pedFcwCheckPedVclOverlap()
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;

  PedFCW::pedFcwCheckPedVclOverlap();
}


extern "C" void SEP_pedFcwCopyDropPoints()
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);

  RETURN_IF_MAIN_PEDS_DISABLED;

  PedFCW::pedFcwCopyDropPoints();
}
extern "C" void SEP_pedFcwObjectsUpdate ()
{
	if (!CommonResources::instance().isFCWSetup()){
		return;
	}
	RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
	RETURN_IF_MAIN_PEDS_DISABLED;
	PedFCW::pedFCWPedObjectsSync();
}

extern "C" void SEP_pedFcwFrameUpdateNoRadarFusion ()
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  //  if (RadarMatching_API::getMode() != Radar::e_FUSION) {
    pedFcwFrameUpdate();
  //}
}

extern "C" void SEP_pedFcwFrameUpdatePedObj ()
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::pedFCWPedObjUpdate();
}

extern "C" void SEP_cvFcwUpdateSignals () {
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  CVFCW::updateSignals();
}

extern "C" void SEP_cvFcwObjectsUpdate (int instIdx)
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  CVFCW::syncVD3DObjects(instIdx);
  CVFCW::syncFusionObjects(instIdx);
}

extern "C" void SEP_cvFcwFrameUpdate (int instIdx)
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  CVFCW::calcCVFcwFunction(instIdx);
}

extern "C" void SEP_cvFcwfillIF()
{
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  CVFCW::fillIF();
}

extern "C" void SEP_pedFcwSetFSData()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }

  RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::setFsData();
}

//This SEP is needed to prevent dependencies between peds SEPs and failSafe SEPs
extern "C" void SEP_pedUpdateFailSafeStatus()
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  PedFCW::updateFailSafeSignals();
  //CVFCW::updateFailSafeSignals();
  PedFCW::logRunFunc("SEP_pedUpdateFailSafeStatus", 0);
}
*/

extern "C" void SEP_findPanicCandidates(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_PEDS_IF_NO_T2(instIdx);
  if(instIdx == SEP::Exp_T1 || instIdx == SEP::Exp_T2){
    Pedestrians_API::prepPanicCandidatesTracking(instIdx);
    Pedestrians_API::postPanicCandidatesTracking(instIdx);
  }
}

extern "C" void SEP_panicNNClassifier(int instIdx){
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
RETURN_PEDS_IF_NO_T2(instIdx);
  if(instIdx == SEP::Exp_T1 || instIdx == SEP::Exp_T2){
    Pedestrians_API::panicNNClassifier(instIdx);
  }

}

extern "C" void SEP_prepPanicLbpclassifier(int instIdx)
{
  /*Empty*/
}

extern "C" void SEP_postPanicLbpclassifier(int instIdx)
{
   /*Empty*/

}

extern "C" void SEP_panicFinScoreAndApprove(int instIdx)
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
RETURN_PEDS_IF_NO_T2(instIdx);
  if(instIdx == SEP::Exp_T1 || instIdx == SEP::Exp_T2){
    Pedestrians_API::panicFinScoreAndApprove(instIdx);
  }
}

/*
extern "C" void SEP_AebHmiFrameUpdate(int instIdx)
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }

  RETURN_IF_MAIN_PEDS_DISABLED;

  if (instIdx==(int)AEB_HMI::RUN_INST_1) {
    PartialRun_API::updatePartialRunLoad(PedFCW::getPedFCWIF_wrapper(), PartialRun::IFCategory::PED_FCW_IF);
  }

  if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::Pedestrians)){
    assert(instIdx==(int)AEB_HMI::RUN_INST_1 || instIdx==(int)AEB_HMI::RUN_INST_2);
    AEB_HMI::updateAEBInputSignals((AEB_HMI::RunningInstance)instIdx);
    AEB_HMI::calcHMI((AEB_HMI::RunningInstance)instIdx);
    AEB_HMI::sendOutput((AEB_HMI::RunningInstance)instIdx);
    if (instIdx==(int)AEB_HMI::RUN_INST_1) {
      int postAEBActivation = -1;
      int postAEB_CRC_Activation = -1;
      AEB_HMI::checkAEBLevelDecision(postAEBActivation, postAEB_CRC_Activation, (AEB_HMI::RunningInstance)instIdx);
      PedFCW::set_AEBAlert_In_PEDFCWIF(postAEBActivation, postAEB_CRC_Activation);
      PartialRun_API::updatePartialRunStore(PedFCW::getPedFCWIF_wrapper(), PartialRun::IFCategory::PED_FCW_IF);
    }
    PedFCW::logRunFunc("SEP_AebHmiFrameUpdate", instIdx);
  }



}

extern "C" void SEP_AebHmiPrepareNextFrame(int instIdx)
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
RETURN_IF_MAIN_PEDS_DISABLED;
  AEB_HMI::prepareNextFrame((AEB_HMI::RunningInstance)instIdx);


}
extern "C" void SEP_AEBHmiVerifyOutputRedundancy()
{
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  //Inside verifyOutputRedundancy performed check of warningLevels from vehicles
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::vehicles);
  RETURN_IF_MAIN_PEDS_DISABLED;
  AEB_HMI::verifyOutputRedundancy();
}




extern "C" void SEP_updateDriverActivity_PEDS_T0(){
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
if (!CommonResources::instance().isFCWSetup()){
  return;
}


  PedFCW::updateDriverActivity(SEP::Exp_T1, 0.0277);

  PedFCW::logRunFunc("SEP_updateDriverActivity_PEDS_T0", 0);
}
extern "C" void SEP_updateDriverActivity_PEDS_C0(){
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
if (!CommonResources::instance().isFCWSetup()){
  return;
}

  PedFCW::updateDriverActivity(SEP::Exp_C1, 0.0277);
  PedFCW::logRunFunc("SEP_updateDriverActivity_PEDS_C0", 0);
}
extern "C" void SEP_updateDriverActivity_PEDS_T1(){
RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
if (!CommonResources::instance().isFCWSetup()){
  return;
}

  PedFCW::updateDriverActivity(SEP::Exp_T2, 0.0277);
  PedFCW::logRunFunc("SEP_updateDriverActivity_PEDS_T1", 0);
}
extern "C" void SEP_updateDriverActivity_PEDS_C1(){
RETURN_IF_MAIN_PEDS_DISABLED;
RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
if (!CommonResources::instance().isFCWSetup()){
  return;
}


  PedFCW::updateDriverActivity(SEP::Exp_C2, 0.0277);
  PedFCW::logRunFunc("SEP_updateDriverActivity_PEDS_C1", 0);
}
*/
extern "C" void SEP_updateSelfShadowStatus(){
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  if(globalFrameIndex < 1){
    return;
  }
  Pedestrians_API::updateSelfShadowStatus(SEP::Exp_T1);
}


extern "C" void SEP_updateFreeSpaceMFAnalyzer(int instIdx){
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }
  if(instIdx == SEP::Exp_T1){
    SceneAnalysis::freeSpaceMFAnalyzer::Instance().update(instIdx);
  }
  if(instIdx == SEP::Exp_T1 && CommonResources::instance().isWideSetup()){
      SceneAnalysis::freeSpaceMFAnalyzer::Instance().updateFarRangeVisibility();
  }
}

extern "C" void SEP_updatePedCliquesInfo(int instIdx)
{
  RETURN_IF_MAIN_PEDS_DISABLED;

  PartialRun_API::updatePartialRunLoad(CommonResources::instance().getPedCliquesInfoWrapper(), PartialRun::IFCategory::PED_CLIQUE_IF);

  if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::Pedestrians)){
    Pedestrians_API::updatePedCliquesInfo();
  }
  PartialRun_API::updatePartialRunStore(CommonResources::instance().getPedCliquesInfoWrapper(), PartialRun::IFCategory::PED_CLIQUE_IF);

}

CameraInfo::CameraInstance pedsGetValidForLCCamera(){
   CameraInfo::CameraInstance  camInst = CameraInfo::e_FORWARD;
   if (
       CameraInfo::exists(camInst) &&
       PrepSys_API::ToneMapLutExists(PrepSys::exp_mask::T0, camInst))
  {
       return camInst;     
  }
  MEImageInfo::ImageInfoGeneral&  imgInfoGeneral =  MEImageInfo::ImageInfoGeneral::instance();
  const std::vector<MEImageInfo::ImageInfoId::ImageKey>& camInstVec = imgInfoGeneral.getImageKeys();
  int numberOfCameras = camInstVec.size();
  
  
  for (int camIndex = 0; camIndex < numberOfCameras ; ++camIndex) {
    camInst = imgInfoGeneral.getCamInstByImageKey(camInstVec[camIndex]);
    if (
        CameraInfo::exists(camInst) &&
        PrepSys_API::ToneMapLutExists(PrepSys::exp_mask::T0, camInst)){
        return camInst;     
    }
  }
  return CameraInfo::CameraInstance::e_INVALID_CAM;
}
extern "C" void SEP_LC_pedsDetectionUpdate(int instIdx) {
  CameraInfo::CameraInstance camInst = pedsGetValidForLCCamera();
  if (camInst !=  CameraInfo::CameraInstance::e_INVALID_CAM){
    MEImageInfo::ImageInfoGeneral&  imgInfoGeneral =  MEImageInfo::ImageInfoGeneral::instance();
    MEImageInfo::ImageInfoId currImageInfo(imgInfoGeneral.getImageKeyByCamInst(camInst),General::GTM,PrepSys::exp_mask::T0);
    LC_API::updatePedsDecision(imgInfoGeneral.getImageInfo(currImageInfo)->getMetadata(),
                              imgInfoGeneral.getImageInfo(currImageInfo)->getTimeStamp(),
                              (*PrepSys_API::getToneMapLut(PrepSys::exp_mask::T0, camInst))->begin(),
                              256,
                              camInst);
  }
}
/*
void readFromLogProcess() {
  RETURN_IF_MAIN_PEDS_DISABLED;
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!CommonResources::instance().isFCWSetup()){
    return;
  }

  PedsWSTester::RunFlowReader* rfReader =  PedFCW::getRunFlowReader();
  PedsWSTester::RF_FUNC rf_Func;
  int gfi = 0;
  int baseGI =CommonResources::instance().currGrabIndex(SEP::Exp_T1);
  int grabIndex= -4;
  int val;
  globalFrameIndex--;
  while (rfReader->readFunction(rf_Func, val)) {
    switch (rf_Func) {
    case PedsWSTester::RF_FUNC::SEP_UPDATE_PLM_STATUS:
      globalFrameIndex++;
      PedFCW::logPLM();
      break;
    case PedsWSTester::RF_FUNC::PED_FCW_FRAME_UPDATE:
      grabIndex++;
      itrkWriter::setCurrentGrabIndex(grabIndex+baseGI,grabIndex+baseGI, true);
      pedFcwFrameUpdate();
     // std::cout<< "pedFcwFrameUpdate()" << std::endl;
      break;
    case PedsWSTester::RF_FUNC::PED_FCW_FRAME_UPDATE_PANIC:
      PedFCW::pedFCWframeUpdatePanicFromLog(val);
      break;
    case PedsWSTester::RF_FUNC::SEP_SETALERTBYVALID:
      SEP_setAlertByValidation(val);
      break;
    case PedsWSTester::RF_FUNC::SEP_PEDVALIDATETTC :
      SEP_pedValidateTTC(val);
      break;
    case PedsWSTester::RF_FUNC::SEP_PEDVALIDATEROI :
      SEP_pedValidateROI(val);
      break;
    case PedsWSTester::RF_FUNC::SEP_PEDVALIDATEPOSITION :
      SEP_pedValidatePosition(val);
      break;
    case PedsWSTester::RF_FUNC::SEP_PEDVALIDATECO :
      SEP_pedValidateCO(val);
      break;
    case PedsWSTester::RF_FUNC::SEP_PEDUPDATEFAILSAFESTATUS :
      SEP_pedUpdateFailSafeStatus();
      break;
    case PedsWSTester::RF_FUNC::SEP_UPDATEDriver_TO:
      grabIndex++;
      itrkWriter::setCurrentGrabIndex(grabIndex+baseGI,grabIndex+baseGI, true);
      SEP_updateDriverActivity_PEDS_T0();
      break;
    case PedsWSTester::RF_FUNC::SEP_UPDATEDriver_CO:
      grabIndex++;
      itrkWriter::setCurrentGrabIndex(grabIndex+baseGI,grabIndex+baseGI, true);
      SEP_updateDriverActivity_PEDS_C0();
      break;
    case PedsWSTester::RF_FUNC::SEP_UPDATEDriver_T1:
      grabIndex++;
      itrkWriter::setCurrentGrabIndex(grabIndex+baseGI, grabIndex+baseGI,true);
      SEP_updateDriverActivity_PEDS_T1();
      break;
    case PedsWSTester::RF_FUNC::SEP_UPDATEDriver_C1:
      SEP_updateDriverActivity_PEDS_C1();
      break;
    case PedsWSTester::RF_FUNC::SEP_AEBHMIFRAMEUPDATE :
      SEP_AebHmiFrameUpdate(val);
      ++gfi;
      itrkWriter::setFrameNum(gfi/2);
      break;
    default:
      break;

    }
  }
  std::cout << "MEST run ended successfully!" << std::endl;
  exit(0);
}
*/

