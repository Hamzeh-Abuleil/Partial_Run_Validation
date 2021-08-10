#include "technology/calibration/utilities/cameraProjections/distortionCorrection_algoAPI.h"
#include "technology/DS/common/SLIinterfaceAPI.h"
#include "technology/brain2/prepSys/common/prepSys_API_Internal.h"
#include "technology/include/SEP/SEP_InstIdx.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "utilities/vehicleInformation/vehicleInformation_API.h"
#include "utilities/realWorld/realWorld_API.h"
#include "technology/mobilib/fix/common/MEXmisc/timeStampUtils.h"
#include "technology/calibration/utilities/cameraProjections/cameraProjectionsAPI.h"
#include "technology/calibration/utilities/cameraProjections/common/initRectifications.h"
#include "utilities/autoFix/autoFix_API.h"
#include "technology/DS/SLIutils/common/SLIUtils.h"
#include "technology/DS/SLIutils/SLIGeneralUtils.h"
#include "technology/DS/SLIutils/TSRPointTracker/common/TSRPointTrackerUtils.h"

#include "functionality/partialRun/partialRun_API.h"
#include "technology/DS/common/SLIinterfaceMemory.h"

#include "technology/road/interface/roadIF.h"
#include "technology/DS/TSRMultiFrame/tsrMF_API.h"

#include <string>
#include "utilities/clipextIO/clipextIO.h"

using namespace ClipextIO;
using namespace std;
using namespace Fix;
using namespace MEimage;
using namespace MEmisc;

extern int globalFrameIndex;
extern int globalBrainIndex;

// for opelGPS
#include "technology/brain2/brain2_API.h"
#include "functionality/interface/modelIF.h"
#include "utilities/vehicleInformation/interface/hostVehicleIF.h"

extern "C" void SEP_updateCommonEgoMotions()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);

  if (!TSR::isTechActive()) {
    return;
  }
  TSR::updateEgoMotion();
}

extern "C" void SEP_updateGPSState()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);

  TSR::updateGPSInfo();
}

extern "C" void SEP_TSRMFStoreRoadT0Model()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);

  TSR::updateLaneCenter();
}

extern "C" void SLIdumpDataUnitTest()
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::TSR);

	CHECK_FORWARD_CAM;
  static bool firstFrame = true;
  static bool dumpVclData = false;
  static bool dumpVD3D = false;
  static bool dumpEgoMotion = false;
  static bool dumpCenterLane = false;
  if (firstFrame) {
    bool dumpTsrUnitTest = Debug::Args::instance().existsParameter("-sdumpTsrUnitTest");
    dumpVclData = Debug::Args::instance().existsParameter("-sdumpVclData") || dumpTsrUnitTest;
    dumpVD3D = Debug::Args::instance().existsParameter("-sdumpVD3D") || dumpTsrUnitTest;
    dumpEgoMotion = Debug::Args::instance().existsParameter("-sdumpEgoMotion") || dumpTsrUnitTest;
    dumpCenterLane = Debug::Args::instance().existsParameter("-sdumpCenterLane") || dumpTsrUnitTest;
  }

  //dump vcl
  if (dumpVclData){
    static ClipextWriter vclWriter(".vcl");
    vclWriter.setExpID(CEXT_SLOW);
    const MEtypes::ptr_vector<Float::MEgeo::AttentionRect> & rects = TSR::getVclCandidates();
    for (unsigned int i=0; i < rects.size(); i++) {
      float conf = rects[i].getConfidence();
      bool valid = rects[i].valid();
      Float::MEgeo::Rect rect = rects[i];
      vclWriter.setData("rect", &rect);
      vclWriter.setData("conf",&conf);
      vclWriter.setData("valid",&valid);
      vclWriter.flushTuple();
    }
  }

  //dump vd3d
  if (dumpVD3D){
    static ClipextWriter vd3dWriter(".vd3d");
    vd3dWriter.setExpID(CEXT_SLOW);
    const auto& vd3dObstacles = TSR::getVd3dCandidates();
    for (const auto& vd3d : vd3dObstacles) {
      vd3dWriter.setData("vdAngleRelativeHost",&vd3d.vdAngleRelativeHost);
      vd3dWriter.setData("camera", (int*)&vd3d.imageKey);
      vd3dWriter.setGeneral("visibleRect", &vd3d.visibleRect);
      vd3dWriter.setData("vdRect", &vd3d.vdRect);
      vd3dWriter.setGeneral("visibleSide", &vd3d.visibleSide);
      vd3dWriter.setGeneral("left",&vd3d.left);
      vd3dWriter.setGeneral("right",&vd3d.right);
      vd3dWriter.setData("sideRect",&vd3d.sideRect);
      vd3dWriter.flushTuple();
    }
  }

  if (dumpEgoMotion) {
    static ClipextWriter egoMotionWriter(".egoMotion");
    for (auto imageKey : TSR::getActiveImageKeys()) {
      CameraInfo::CameraInstance camera = TSR::getCameraInstance(imageKey);
      const Float::MEmath::Mat<4, 4, double>* egoMotionEM = TSR::getEgoMotion(imageKey,TSR::ROAD_EM_MOTION);
      bool validRoadEM = egoMotionEM != NULL;
      egoMotionWriter.setExpID(ClipextIO::CEXT_SLOW,camera);
      egoMotionWriter.setData("road_valid",&validRoadEM);
      if (validRoadEM) {
        egoMotionWriter.setData("road_em",egoMotionEM);
      }
      else {
        Float::MEmath::Mat<4, 4, double> tmp; //zeros filled
        egoMotionWriter.setData("road_em",&tmp);
      }

      const Float::MEmath::Mat<4, 4, double>* egoMotionVehicle = TSR::getEgoMotion(imageKey,TSR::VEHICLE_MOTION);
      bool validVehicleEM = egoMotionVehicle != NULL;
      egoMotionWriter.setData("vehicleInfo_valid",&validVehicleEM);
      if (validVehicleEM) {
        egoMotionWriter.setData("vehicleInfo_em",egoMotionVehicle);
      }
      else {
        Float::MEmath::Mat<4, 4, double> tmp; //zeros filled
        egoMotionWriter.setData("vehicleInfo_em",&tmp);
      }

      const Float::MEmath::Mat<4, 4, double>* egoMotionYawPitch = TSR::getEgoMotion(imageKey,TSR::YAW_PITCH_MOTION);
      bool validYawPitch = egoMotionYawPitch != NULL;
      egoMotionWriter.setData("yawPitch_valid",&validYawPitch);
      if (validYawPitch) {
        egoMotionWriter.setData("yawPitch_em",egoMotionYawPitch);
      }
      else {
        Float::MEmath::Mat<4, 4, double> tmp; //zeros filled
        egoMotionWriter.setData("yawPitch_em",&tmp);
      }
      egoMotionWriter.flushTuple();
    }
  }

  if (dumpCenterLane) {
    static ClipextWriter centerLaneWriter(".centerLane");
    centerLaneWriter.setExpID(ClipextIO::CEXT_SLOW);
    const auto& laneCenter = TSR::getLaneCenter();
    centerLaneWriter.setData("laneCenterValid",&laneCenter.valid);
    centerLaneWriter.setData("centerModel",laneCenter.laneCenterModel,4);
    centerLaneWriter.flushTuple();
  }

  firstFrame = false;
}

extern "C" void SEP_SLIendOfFrame(int instIdx)
{
  PartialRun_API::updatePartialRunLoad(SLIinterfaceMemory::instance().getSliIF_wrapper(), PartialRun::IFCategory::DS_SLI_IF_END_FRAME);
  if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::TSR)){
  if (!TSR::isTechActive()) {
      return;
    }
  TSR::endFrameDS();
    if (Brain2API::getModelIF()->hostVehicle != NULL) {
      SLIinterfaceAPI::setOpelGPS(&Brain2API::getModelIF()->hostVehicle->opelGPS);
    }
    SLIinterfaceAPI::endOfSLIFrame();
    TSR::API::updateItrk();

  #ifdef OS_FAMILY_UNIX
    SLIdumpDataUnitTest();
  #endif
  }
  PartialRun_API::updatePartialRunStore(SLIinterfaceMemory::instance().getSliIF_wrapper(), PartialRun::IFCategory::DS_SLI_IF_END_FRAME);

}
