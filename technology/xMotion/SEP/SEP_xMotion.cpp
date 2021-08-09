#include "technology/include/SEP/SEP_InstIdx.h"
#include "technology/xMotion/EyeQ2/VMP_RPC_xTracker.h"
#include "technology/xMotion/EyeQ2/VMP_RPC_crossing_edge_mask.h"
#include "technology/xMotion/common/xMotion_API_Internal.h"
#include "technology/xMotion/common/verticalEdges_API_internal.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/mobilib/fix/common/MEXmisc/timeStampUtils.h"
#include "technology/xMotion/xmotionImageInfo.h"
#include "functionality/partialRun/partialRun_API.h"

//gsf_task_def prepareCrossingXTracker[3];
//gsf_task_def prepareCrossingXTrackerT1T0[3];

gsf_task_def crossingXTracker1;
gsf_task_def crossingXTracker2;
gsf_task_def crossingXTracker3;
gsf_task_def crossingXTrackerT1T0_1;
gsf_task_def crossingXTrackerT1T0_2;
gsf_task_def crossingXTrackerT1T0_3;

gsf_task_def verticalEdges_0;
gsf_task_def verticalEdges_1;
gsf_task_def verticalEdges_2;
gsf_task_def verticalEdges_3;

extern int globalFrameIndex;
extern "C" void SEP_initXMotionStructs(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::initXMotionStructs();
}

extern "C" void SEP_prepareCrossingXTracker(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  MEImageInfo::ImageKey cinst = MEImageInfo::IMAGE_REGULAR;
  // VE will run on the fish-eye image if exists
  if (MEImageInfo::ImageInfoxMotion::instance().imageExists(MEImageInfo::IMAGE_WIDE)) {
    cinst = MEImageInfo::IMAGE_WIDE;
    assert(CameraInfo::cameraPlacement(MEImageInfo::ImageInfoxMotion::instance().getImageInfoBase({cinst, xMotion::RAW, 0})->getCamInst()) == Fix::MEstd::FORWARD_FISHEYE);
  }
  const auto imgInfo0 = MEImageInfo::ImageInfoxMotion::instance().getImageInfo({cinst, xMotion::LTM, 0});
  const auto imgInfo1 = MEImageInfo::ImageInfoxMotion::instance().getImageInfo({cinst, xMotion::LTM, 1});

  const Prep::SafeImgPyr* interPyer  =   imgInfo1->getPyramid(-1);
  const Prep::SafeImgPyr* vclPrevPyr = imgInfo0->getPyramid(-1);
  const Prep::TimeStampHist* timeStampT1 = imgInfo0->getTimeStamp();
  const Prep::TimeStampHist* timeStampT2 = imgInfo1->getTimeStamp();
  
  float prevDTime,currDTime,dTimeRatio;

  dTimeRatio=1;
  if(globalFrameIndex>2){
    globalFrameIndex==0 ? prevDTime=0 : prevDTime=dTime(*timeStampT1,-2,-1);
    currDTime = dTime(*timeStampT1,-1,*timeStampT2,-1);
    
    assert(currDTime>0);
    prevDTime<=0 ? dTimeRatio=1 : dTimeRatio = currDTime/prevDTime;
  }
  gsf_task_id currTask = &crossingXTracker3;
  if (instidx==2){
	  currTask = &crossingXTracker2;
  }else if (instidx==1){
	  currTask = &crossingXTracker1;
  }
  XMotion::prepareCrossingXTracker(instidx,vclPrevPyr,interPyer,dTimeRatio, currTask);
}

extern "C" void SEP_prepareCrossingXTrackerT1T0(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  MEImageInfo::ImageKey cinst = MEImageInfo::IMAGE_REGULAR;
  // VE will run on the fish-eye image if exists
  if (MEImageInfo::ImageInfoxMotion::instance().imageExists(MEImageInfo::IMAGE_WIDE)) {
    cinst = MEImageInfo::IMAGE_WIDE;
    assert(CameraInfo::cameraPlacement(MEImageInfo::ImageInfoxMotion::instance().getImageInfoBase({cinst, xMotion::RAW, 0})->getCamInst()) == Fix::MEstd::FORWARD_FISHEYE);
  }
  const auto imgInfo0 = MEImageInfo::ImageInfoxMotion::instance().getImageInfo({cinst, xMotion::LTM, 0});
  const auto imgInfo1 = MEImageInfo::ImageInfoxMotion::instance().getImageInfo({cinst, xMotion::LTM, 1});

  const Prep::SafeImgPyr* interPyer  = imgInfo1->getPyramid(-1);
  const Prep::SafeImgPyr* pedCurrPyr = imgInfo0->getPyramid(0);
  const Prep::TimeStampHist* timeStampT1 = imgInfo0->getTimeStamp();
  const Prep::TimeStampHist* timeStampT2 = imgInfo1->getTimeStamp();;

  float prevDTime,currDTime,dTimeRatio;

  dTimeRatio=1;
  if(globalFrameIndex>2){
    globalFrameIndex==0 ? prevDTime=0 : prevDTime=dTime(*timeStampT1,-1,*timeStampT2,-1);
    currDTime = dTime(*timeStampT2,-1,*timeStampT1,0);
    assert(currDTime>0);
    prevDTime<=0 ? dTimeRatio=1 : dTimeRatio = currDTime/prevDTime;
  }

  gsf_task_id currTask = &crossingXTrackerT1T0_3;
  if (instidx==2){
	  currTask = &crossingXTrackerT1T0_2;
  }else if (instidx==1){
	  currTask = &crossingXTrackerT1T0_1;
  }
  XMotion::prepareCrossingXTracker(instidx,interPyer,pedCurrPyr,dTimeRatio, currTask);
}
extern "C" void SEP_postCrossingXTracker(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::postCrossingXTracker(instidx,false);
}

extern "C" void SEP_postCrossingXTrackerT1T0(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::postCrossingXTracker(instidx, true);
}
extern "C" void SEP_crossingMain(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::crossingMain();
}

extern "C" void SEP_vehicleAttentionOnly(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::vehicleAttentionOnly();
}


extern "C" void SEP_prepareFindNewEdges(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  switch (instidx) {
  case SEP::VEXMOTION_FENCE:
    XMotion::prepareFindNewEdges_FenceAlgo();
    break;
  default:
    XMotion::prepareFindNewEdges();
  }
}


extern "C" void SEP_postFindNewEdges(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  switch (instidx) {
  case SEP::VEXMOTION_FENCE:
    XMotion::postFindNewEdges_FenceAlgo();
    break;
  default:
    XMotion::postFindNewEdges();
  }
}


extern "C" void INIT_xMotionInitVMP(int instidx)
{
  CHECK_FORWARD_CAM
  VMP_xTracker_init();
}


// vertical edges goals
extern "C" void SEP_prepareVerticalEdgesL0(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::prepareVerticalEdges(instidx,0, &verticalEdges_0);
}

extern "C" void SEP_prepareVerticalEdgesL1(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::prepareVerticalEdges(instidx,1,&verticalEdges_1);
}

extern "C" void SEP_prepareVerticalEdgesL2(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::prepareVerticalEdges(instidx,2,&verticalEdges_2);
}

extern "C" void SEP_prepareVerticalEdgesL3(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::prepareVerticalEdges(instidx,3,&verticalEdges_3);
}

extern "C" void SEP_postVerticalEdges(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::postVerticalEdges(instidx);
}

extern "C" void SEP_findEdgesFatherAndInherit(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  switch (instidx) {
  case SEP::VEXMOTION_FENCE:
    XMotion::finalizeEdgesWithNoFatherAndInherit(instidx);
    break;
  default:
    XMotion::findEdgesFatherAndInherit(instidx);
  }
}

extern "C" void INIT_crossingEdgeMaskVMP(int instidx)
{
  CHECK_FORWARD_CAM
  VMP_crossing_edge_mask_init();
}

extern "C" void SEP_emptyXmotion(int instidx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::xMotion);
  CHECK_FORWARD_CAM
  XMotion::emptyXmotion();
}
