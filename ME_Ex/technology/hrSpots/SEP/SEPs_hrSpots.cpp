#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/hrSpots/images/exposures.h"
#include "technology/hrSpots/images/diCoordinates.h"
#include "technology/hrSpots/subFrames.h"
#include "technology/hrSpots/API.h"
#include "technology/hrSpots/agendasSynchronizer.h"
#include "technology/hrSpots/sharedData/common/API_internal.h"
#include "technology/hrSpots/report/API.h"
#include "technology/hrSpots/attention/API.h"
#include "technology/hrSpots/tracking/API.h"
#include "technology/hrSpots/sceneDetection/API.h"
#include "technology/hrSpots/features/API.h"
#include "technology/hrSpots/couples/API.h"
#include "technology/hrSpots/onlineCalibration/API.h"
#include "technology/brain2/prepSys/common/imageCollection.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/brain2/brain2_API.h"
#include "functionality/frameMetadata/HRCCMetadata.h"
#include "utilities/electricGrid/electricGrid_API.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "utilities/lightingConditions/lightingConditions_API.h"
#include "utilities/autoFix/autoFix_API.h"
#include "technology/objectSensing/objectDetection/VD2D/common/vclCommonResources.h"


#include "functionality/partialRun/partialRun_API.h"
#include "functionality/partialRun/wrappers/HrSpotsSharedData_wrapper.h"
#include "technology/brain2/brain2_API.h"
#include "functionality/interface/modelIF.h"
#include "functionality/partialRun/wrappers/HRSpotsIF_wrapper.h"

#define RUN_DEBUG_CLEAR_MF
#undef RUN_DEBUG_CLEAR_MF

#ifdef RUN_DEBUG_CLEAR_MF
bool clearMFDebug();
#define DEBUG_CLEAR_MF() {if(clearMFDebug()) return;}
#else
#define DEBUG_CLEAR_MF()
#endif

#define CHECK_FORWARD_CAM if(!CameraInfo::exists(CameraInfo::e_FORWARD)) {return;}
#define DISABLE if(hrSpots::HLBProperties_API::no_hlb() || !CameraInfo::exists(CameraInfo::e_FORWARD)) {return;}

#define CHECK_REAR_CAM if(CameraInfo::cameraPlacement()==Fix::MEstd::REAR) {return;}

gsf_task_def hrSpotsBrightSceneHist_exp0;
gsf_task_def hrSpotsBrightSceneHist_exp1;


using namespace hrSpots::images;

namespace hrSpots
{
  namespace images
  {

    void initFrame(int subFrame)
    {
      DEBUG_CLEAR_MF();
      DISABLE;
      using hrSpots::images::SubFrames;

      const dstruct_t* metadata = NULL; 
      const MEImageInfo::ImageInfoRepo * imageInfo = hrSpots::images::HLBImageInfo::instance();

      if (subFrame == SubFrames::COLOR0) {
        metadata = imageInfo->getImageInfoBase({MEImageInfo::IMAGE_REGULAR, hrSpots::RAW, 0})->getMetadata();
        hrSpots::images::initExposureFrame(metadata, Agenda::PROCESS, SubFrames::TEXTURE0); 
        metadata = imageInfo->getImageInfoBase({MEImageInfo::IMAGE_REGULAR, hrSpots::RAW, 1})->getMetadata();
        hrSpots::images::initExposureFrame(metadata, Agenda::PROCESS, SubFrames::COLOR0); 
      }
      else {
        assert(subFrame == SubFrames::COLOR1);
        metadata = imageInfo->getImageInfoBase({MEImageInfo::IMAGE_REGULAR, hrSpots::RAW, 2})->getMetadata();
        hrSpots::images::initExposureFrame(metadata, Agenda::PROCESS, SubFrames::TEXTURE1); 
        metadata = imageInfo->getImageInfoBase({MEImageInfo::IMAGE_REGULAR, hrSpots::RAW, 3})->getMetadata();
        hrSpots::images::initExposureFrame(metadata, Agenda::PROCESS, SubFrames::COLOR1); 
      }

    }

    void initFrameReport(int subFrame)
    {
      DEBUG_CLEAR_MF();
      DISABLE;
      using hrSpots::images::SubFrames;

      ASSERT( subFrame < SubFrames::COUNT_REPORT && subFrame > -1);
      const dstruct_t* metadata = NULL;
      const MEImageInfo::ImageInfoRepo * imageInfo = hrSpots::images::HLBImageInfo::instance();
      switch (subFrame) {
      case (SubFrames::TEXTURE0) :
        metadata = imageInfo->getImageInfoBase({MEImageInfo::IMAGE_REGULAR, hrSpots::RAW, 0})->getMetadata();
        break;
      case (SubFrames::COLOR0) :
        metadata = imageInfo->getImageInfoBase({MEImageInfo::IMAGE_REGULAR, hrSpots::RAW, 1})->getMetadata();
        break;
      case (SubFrames::TEXTURE1) :
        metadata = imageInfo->getImageInfoBase({MEImageInfo::IMAGE_REGULAR, hrSpots::RAW, 2})->getMetadata();
        break;
      case (SubFrames::COLOR1) :
        metadata = imageInfo->getImageInfoBase({MEImageInfo::IMAGE_REGULAR, hrSpots::RAW, 3})->getMetadata();
        break;
      default:
        assert(0);
      }      
      

      hrSpots::images::initExposureFrame(metadata, Agenda::REPORT, subFrame); 
      hrSpots::report::API::initFrame(); 
    }

  }
}


extern "C"
{
  void SEP_hrSpotsUpdateAutofix()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::images::updateAutofix();
  }
  
  void SEP_hrSpotsUpdateSharedDIRect()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;

    hrSpots::images::initExposureRectsFrame();    
  }

  void SEP_hrSpotsUpdateEgoSpeed(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;

    if (subFrame==hrSpots::images::SubFrames::COLOR0) {

      hrSpots::API::updateEgoSpeed(subFrame);
      hrSpots::features::API::initFrame();
    }   
  }

  

  // ----------  synchronizer process/thread -------------
  void SEP_hrSpotsInitModelReportFrame(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;

    hrSpots::AgendasSynchronizer::initFrame(subFrame);
  }

  void SEP_hrSpotsInitOnlineCalibrationReport(int subFrame) {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
  hrSpots::AgendasSynchronizer::copyOnlineCalibrationDecisions();
  }
  
  void SEP_hrSpotsInitModelReport(int subFrame) {
    DEBUG_CLEAR_MF();
        DISABLE;
    PartialRun_API::updatePartialRunLoad(hrSpots::AgendasSynchronizer::getModelWrapperIF(), PartialRun::IFCategory::HRSPOTS_IF_FL);

    if(!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::hrSpots))
    {   
    hrSpots::AgendasSynchronizer::copyProcessData(subFrame);
    hrSpots::AgendasSynchronizer::updateModelIFManager(subFrame);
    // the decision update to itrk 9 must be called right after updateHighLowBeam where 2step decisions are updated
    hrSpots::AgendasSynchronizer::copyDecision2Itrk9();
    }

    PartialRun_API::updatePartialRunStore(hrSpots::AgendasSynchronizer::getModelWrapperIF(), PartialRun::IFCategory::HRSPOTS_IF_FL);

  }

  // ----------------  report  ----------------------
  void SEP_hrSpotsInitFrameReport(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::images::initFrameReport(subFrame);
  }
  
  void SEP_hrSpotsInitFrame(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    // TODO : separate goal initExposures (C0 + C1) from others (only C0)
    if (subFrame==hrSpots::images::SubFrames::COLOR0) {
      ElectricGridAPI::updateElectricGrid();
    }
    hrSpots::images::initFrame(subFrame);
    hrSpots::API::initFrame(subFrame);
  }
  
  void SEP_hrSpotsPredictReportModel(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::report::API::predict(subFrame);
  }

  void SEP_hrSpotsPostRawPredictPosition (int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::report::API::postRawPredictPosition(subFrame);
  }

  void SEP_hrSpotsUpdateApprovedVdObjects(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    auto * vmApi=hrSpots::API::getHrUser().get<VD3D_MEAS_API::VD3DMeasurementsService_API>();
    assert(vmApi!=nullptr);
    const Fix::MEimage::Sync<Log::VD3DCollectionIF>* vd3dCollectionIFSync = vmApi->getPredictedObstaclesFA();
    if ((vd3dCollectionIFSync != NULL) && (vd3dCollectionIFSync->available())) {
      const Log::VD3DCollectionIF& vd3DIF = **vd3dCollectionIFSync;
      hrSpots::report::API::updateApprovedVdObjects(vd3DIF);
    }
  }

  // ----------------  attention  ----------------------

  void SEP_hrSpotsAttentionInitFrame(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::initFrame(expInd);
  }


  void SEP_prephrSpotsFindLightCandidates(int expInd)               // prep
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::prepFindLightCandidates(expInd);
  }

  void SEP_posthrSpotsFindLightCandidates(int expInd)               // post
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::postFindLightCandidates(expInd);
  }

  void SEP_toneCandsToRawStrips(int expInd)               
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::toneCandsToRawStrips(expInd);
  }

  void SEP_prepRawLightCandidates(int expInd)               // prep
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::prepRawLightCandidates(expInd);    
  }

  void SEP_postRawLightCandidates(int expInd)               // post
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;

    hrSpots::attention::API::postRawLightCandidates(expInd);
  }

  void SEP_hrSpotsBuildSpots(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::buildSpots(expInd);
  }

  void SEP_hrSpotsFilterBadSpots(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::eraseOverlappingSpots(expInd);
    hrSpots::attention::API::eraseWeakSpots(expInd);
  }
  
  void SEP_hrSpotsCalcSFCheapFeatures(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::calcCheapFeatures(expInd);
  }

  void SEP_hrSpotsCalcSFRedness(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::calcSFRedness(expInd);
  }

  void SEP_hrSpotsPostCalcSFFlatness(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::postCalcSFFlatness(expInd);
  }

  void SEP_hrSpotsFinalizeSpots(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::finalize(expInd);
  }
  
  void SEP_hrSpotsClusterSpots(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::attention::API::clusterSpots(expInd);
  }
  
  // ----------------------------------------------------
  
  // ----------------  tracking -------------------------
  
  void SEP_hrSpotsTrackingInitFrame(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::tracking::API::initFrame(subFrame);
  }

  void SEP_hrSpotsTrackingComputeSearchRectForMFSpots(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::tracking::API::computeSearchRectForMFSpots(subFrame);
  }

  void SEP_hrSpotsTrackingBeforePitch(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::tracking::API::prePitch(subFrame);
  }

  void SEP_hrSpotsTrackingCalcPitch(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::tracking::API::calcPitch(subFrame);
  }

  void SEP_hrSpotsTrackingAfterPitch(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::tracking::API::postPitch(subFrame);
  }

  void SEP_hrSpotsTrackingMatch(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::tracking::API::match(subFrame);
  }

  // For VD
  void SEP_hrSpotsPredictSharedDataT0Positions()
  {
    DEBUG_CLEAR_MF();
        DISABLE;
    PartialRun_API::updatePartialRunLoad(hrSpots::sharedData::API::getHrSpotsShareDataWrapper(), PartialRun::IFCategory::HRSPOTS_SHARED_DATA_PR);

    if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::hrSpots)){
          hrSpots::sharedData::API::predictT0PositionsAndRects();
        }

    PartialRun_API::updatePartialRunStore(hrSpots::sharedData::API::getHrSpotsShareDataWrapper(), PartialRun::IFCategory::HRSPOTS_SHARED_DATA_PR);
  }
  
   void SEP_hrSpotsPrepareSharedDataSlow(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;

    hrSpots::sharedData::API::initFrameSlow();
    hrSpots::sharedData::API::prepareSharedDataSlow();
  }

   void SEP_hrSpotsPrepareSharedData(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;


    hrSpots::sharedData::API::initFrame();
    hrSpots::sharedData::API::prepareSharedData();        
    
  }

  // TFL matching
  void SEP_hrSpotsMatchTFL(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::API::matchTFL();
  }

  // TSR matching
  void SEP_hrSpotsMatchTSR(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DISABLE;
    DEBUG_CLEAR_MF();
    hrSpots::API::matchTSR(subFrame);
  }

  // Spots line
  void SEP_hrSpotsSpotsLines(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::API::updateSpotsLines();
  }

  void SEP_hrSpotsMfDecision(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::API::mfDecision();
  }

  void SEP_hrSpotsUpdateSpotVdId(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::API::updateSpotVdId();
  }

  void SEP_hrSpotsUpdateReportDistance(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::report::API::updateReportDistance(subFrame);

  }

  void SEP_hrSpotsClearDeadSpotsList(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::tracking::API::clearKilledIDlist();
  }

  // ----------------------------------------------------
  // features

  void SEP_hrSpotsCalcFeatures(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::calculate(subFrame);
  }

  void SEP_hrSpotsPrepareOncomingReflections(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
            
    hrSpots::features::API::prepareOncomingReflections(subFrame);
  }

  void SEP_hrSpotsRetrieveOncomingReflections(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::retrieveOncomingReflections(subFrame);
  }



  // ----------------------------------------------------
  // classification
  void SEP_hrSpotsClassifySpots(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::classifySpots(subFrame);
  }
  void SEP_hrSpotsCollectClassifierFeatures(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::collectClassifierFeatures(subFrame);
  }
  
  void SEP_hrSpotsXYClassify(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::classifyXY(subFrame);
  }
  
  // ---------------------------------------------------
  // Feature dependant of classification (in cascade)

  void SEP_hrSpotsPrepShapeLargeRect(int)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::prepareShapeLargeRect();
  }

  void SEP_hrSpotsPrepShapeLargeRect6_6(int)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::prepareShapeLargeRect6_6();
  }
  void SEP_hrSpotsPrepShapeLargeRect10_10(int)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::prepareShapeLargeRect10_10();
  }

  void SEP_hrSpotsPostShapeLargeRect6_6(int)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::postShapeLargeRect6_6();
  }
  void SEP_hrSpotsPostShapeLargeRect10_10(int)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::postShapeLargeRect10_10();
  }
  
  void SEP_hrSpotsPrepKoreaReflectorsVhfb8_16(int)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::prepareKoreaReflectorsVhfb8_16();
  }
  void SEP_hrSpotsPostKoreaReflectorsVhfb8_16(int)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::postKoreaReflectorsVhfb8_16();
  }

  // ----------------------------------------------------
  // couples
  void SEP_hrSpotsRemoveCouplesWithDeadSpots(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::couples::API::removeCouplesWithDeadSpots();
    hrSpots::couples::API::removeClearacnesWithDeadSpots();
    hrSpots::couples::API::syncKilledSpotsApproval(subFrame);
  }

  void SEP_hrSpotsCouplesInitFrame(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::couples::API::initFrame(subFrame);
  }

  void SEP_hrSpotsCouplesCandidates(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::couples::API::createCoupleCandidates(subFrame);
  }

  void SEP_hrSpotsClearancesCandidates(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::couples::API::createClearanceCandidates(subFrame);
  }

  void SEP_hrSpotsMergeSFcouplesAndClearances(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::couples::API::mergeSFcouplesAndClearances();
  }

  void SEP_couplesCNNclassifiers(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::couples::API::createCNNRects();
    hrSpots::couples::API::classifyCNN();
  }

  void SEP_hrSpotsTrackCouples(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::couples::API::trackCouples();
  }

  void SEP_hrSpotsTrackClearances(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::couples::API::trackClearances();
  }

  void SEP_hrSpotsMergeMFcouplesAndClearances(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::couples::API::mergeMFcouplesAndClearances();
  }

  void SEP_hrSpotsVDMatching(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::couples::API::matchVDData(subFrame);
    hrSpots::couples::API::calcDistances();
  }

  void SEP_couplesDecisionClassifiers(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::couples::API::classifyDecision();
  }

  void SEP_hrSpotsCouplesDecision(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::couples::API::takeDecisions();
    hrSpots::couples::API::syncSpotsAndCouples(subFrame);
  }

  void SEP_hrSpotsUpdateVdId(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::API::updateVdIdsC1();
  }

  void SEP_hrSpotsSyncVehicles(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::API::buildSFCands(subFrame);
    hrSpots::API::buildMFCands();
  }

  // TODO: Remove next SEP
  void SEP_hrSpotsSyncApprovedVdObjects(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    // hrSpots::couples::API::syncApprovedVdObjects();
  }

  // ----------------------------------------------------

  // sceneDetection

  void SEP_hrSpotsUpdateRunningMode(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::updateRunningMode(); 
  }

   void SEP_hrSpotsUpdateSharedRunningMode(int subFrame)
  {
    DEBUG_CLEAR_MF();
        DISABLE;

    PartialRun_API::updatePartialRunLoad(hrSpots::sharedData::API::getHrSpotsSharedRunningModeWrapper(), PartialRun::IFCategory::HRSPOTS_SHARED_RUNNING_MODE_PR);

    if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::hrSpots)){
           hrSpots::sharedData::API::updatePublicRunningMode();  
        }

    PartialRun_API::updatePartialRunStore(hrSpots::sharedData::API::getHrSpotsSharedRunningModeWrapper(), PartialRun::IFCategory::HRSPOTS_SHARED_RUNNING_MODE_PR);
  }

  void SEP_hrSpotsPreparedSharedSceneDetection()
  {
    DEBUG_CLEAR_MF();
        DISABLE;

     PartialRun_API::updatePartialRunLoad(hrSpots::sharedData::API::getHrSpotsSharedSceneDetectionWrapper(), PartialRun::IFCategory::HRSPOTS_SHARED_SCENE_DETECTION_PR);

    if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::hrSpots)){
          hrSpots::sharedData::API::prepareSharedSceneDetection(); 
        }

    PartialRun_API::updatePartialRunStore(hrSpots::sharedData::API::getHrSpotsSharedSceneDetectionWrapper(), PartialRun::IFCategory::HRSPOTS_SHARED_SCENE_DETECTION_PR);
  
  }

  void SEP_hrSpotsReflectorScene(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::updateReflectorsScene(subFrame);
  }

  
  void SEP_hrSpotsPrepareBrightSceneHistograms(int expInd)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    gsf_task_id task_id = &hrSpotsBrightSceneHist_exp1;
    if (expInd == 0){
    	task_id = &hrSpotsBrightSceneHist_exp0;
    }
    hrSpots::sceneDetection::API::prepareBrightSceneHistograms(expInd, task_id);
  }
  void SEP_hrSpotsCalcBrightScene()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::calcBrightScene();
  }
  void SEP_hrSpotsUpdateCloseOCInScene()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::updateCloseOCInScene();
  }
  void SEP_hrSpotsPrepareForLightconeScene(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::prepareForLightconeScene(subFrame);
  }
  void SEP_hrSpotsClassifyLightconeScene(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::classifyLightconeScene(subFrame);
  }
  void SEP_hrSpotsFindStreetLights()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::findStreetLights();
  }

  void SEP_hrSpotsUpdateKalmanSL()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::sceneDetection::API::findKalmanStreetLights();
  }


  void SEP_hrSpotsUpdateSLDistanceClassificationData(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::updateSLDistanceClassificationData(subFrame);
  }  
  void SEP_hrSpotsUpdateSingleSpotSceneData(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::updateSingleSpotSceneData(subFrame);
  }
  void SEP_hrSpotsDecideSingleSpotScene()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::decideSingleSpotScene();
  }
  void SEP_hrSpotsDecideLitNight()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::decideLitNight();
  }
  void SEP_hrSpotsUpdateBlinkingObjectsScene(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::updateBlinkingObjects(subFrame);    
  }  

  void SEP_hrSpotsUpdateLoadBalanceDecision()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::updateLoadBalanceDecision();
  }
               

  void SEP_hrSpotsReducedSensitivityScene(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::sceneDetection::API::reducedSensitivityScene(subFrame);
  }

  void SEP_hrSpotsUpdateRedPolesScene() 
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::sceneDetection::API::updateRedPolesScene();
  }

  void SEP_hrSpotsUpdateRedReflectorScene()
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::sceneDetection::API::updateRedReflectorLine();
  }

  void SEP_hrSpotsUpdateDriverProfileInput(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::API::updateDriverProfileInput(Brain2API::getDriverProfileInput());    
  }
 
  // cnn classifiers
  void SEP_sfSpotsCNNclassifiers(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::createCNNRects(subFrame);
    hrSpots::features::API::classifySpotsCNN(subFrame);
  }

  void SEP_hrSpotsDarkSceneClassifiers(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::features::API::classifyDarkScene();
  }

  void SEP_hrSpotsKalmanPred(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
    DISABLE;
    hrSpots::features::API::updateKalman();
  }

  // online calibration
  void SEP_hrSpotsOnlineCalibrationInitFrame(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::onlineCalibration::API::initFrame(subFrame);
  }

  void SEP_hrSpotsRunOnlineCalibrationCalcCannyEdges(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::onlineCalibration::API::calcCannyEdges(subFrame);
  }

  void SEP_hrSpotsRunOnlineCalibrationSFRaw(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::onlineCalibration::API::runSFRaw(subFrame);
  }

  void SEP_hrSpotsRunOnlineCalibrationSFRefineL0(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::onlineCalibration::API::runSFRefineL0(subFrame);
  }

  void SEP_hrSpotsRunOnlineCalibrationSFRefineLM1(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::onlineCalibration::API::runSFRefineLM1(subFrame);
  }

  void SEP_hrSpotsRunOnlineCalibrationSFPrincipalPoints(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::onlineCalibration::API::runSFPrincipalPointsCalculation(subFrame);
  }

  void SEP_hrSpotsRunOnlineCalibrationMF(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::onlineCalibration::API::runMultiFrame(subFrame);
  }

  void SEP_hrSpotsPublishOnlineCalibrationReport(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::report::API::publishOnlineCalibrationReport(subFrame);
  }

  // house-keeping
  
  void SEP_hrSpotsPublish(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    hrSpots::API::publish(subFrame);
  }

  void SEP_hrSpotsPublishReport(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;

    hrSpots::report::API::publish(subFrame);
  }

  void SEP_hrSpotsPublishReportStrongRefs(int subFrame)
  {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    if ((Brain2API::getModelIF()->sli != NULL) && ((*Brain2API::getModelIF()->sli).available())){
      const SLI_IF& sliIF = *(*Brain2API::getModelIF()->sli);
      hrSpots::report::API::publishStrongRefs(subFrame, sliIF);
    }
  }
  
  void SEP_hrSpotsGetREData(int subFrame) {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    if (!hrSpots::HLBProperties_API::get()->USE_ROAD_EDGE_FOR_HIGHWAY()){return;}
    hrSpots::API::initFrameREData();
    Fix::MEimage::Sync<REFSIF> * refsSync = REFS_API::getREFSoutputModel(CameraInfo::e_FORWARD);
    auto * dsApi = hrSpots::API::getHrUser().get<DS_API::DSService_API>();
    const REFSIF* refsIf = NULL;
    if ((refsSync != NULL) && refsSync->available() && dsApi !=NULL && dsApi->getSLD_IF()!=NULL) {
      const Fix::MEimage::Sync<SemanticLanesDescription::SemanticLanesDescriptionIF>* sldSync = dsApi->getSLD_IF();
      const SemanticLanesDescription::SemanticLanesDescriptionIF* sldIf = &(sldSync->getObj());
      refsIf = &(refsSync->getObj());
      hrSpots::API::updateREData(refsIf,sldIf, subFrame); 
    }
  }
  void SEP_hrSpotsGetNSSData(int subFrame) {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::hrSpots);
    DEBUG_CLEAR_MF();
        DISABLE;
    if (!hrSpots::HLBProperties_API::get()->MATCH_NSS_TYPE_TO_SPOTS()){return;}
    hrSpots::API::initFrameNSSData();
    hrSpots::API::updateNSSData(NSS_API::getSegResults(), subFrame);
    hrSpots::API::matchNSS(subFrame);
  }
}


#ifdef RUN_DEBUG_CLEAR_MF
extern int globalBrainIndex;
extern bool g_gsf_checker_enabled;
bool clearMFDebug()
{
  static int startPowerOffFrame=-1, endPowerOffFrame=-1;
  static int firstFrame = globalBrainIndex;
  if( firstFrame == globalBrainIndex ) {
    int gotName = 0;
    startPowerOffFrame = Debug::Args::instance().getStickyValue("-shrSpotsOffStartFrame",
                                                                startPowerOffFrame,
                                                                &gotName);
    if( gotName > 0) {
      gotName = 0;
      endPowerOffFrame = Debug::Args::instance().getStickyValue("-shrSpotsOffEndFrame",
                                                                endPowerOffFrame,
                                                                &gotName);
      ASSERT(gotName > 0 && "missing end frmae for hrSpots shutoff debug");
      ASSERT(endPowerOffFrame > startPowerOffFrame);      
    }
  }
  ASSERT((startPowerOffFrame == -1 && endPowerOffFrame == -1) ||
         (startPowerOffFrame != -1 && endPowerOffFrame != -1));

  if(startPowerOffFrame == -1) return false;
  else {
    if( globalBrainIndex >= startPowerOffFrame && globalBrainIndex < endPowerOffFrame ) {
      if( globalBrainIndex == (startPowerOffFrame+1) ) {
        g_gsf_checker_enabled = false;
        hrSpots::API::clearMF();
        hrSpots::report::API::clearMF();
        g_gsf_checker_enabled = true;
      }

      return true;
    }
    else
      return false;
  }
}

#endif
