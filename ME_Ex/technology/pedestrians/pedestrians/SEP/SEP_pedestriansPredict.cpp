#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/mobilib/fix/common/MEXmisc/timeStampUtils.h"
#include "technology/pedestrians/pedestrians/common/Pedestrians_API_Internal.h"
#include "technology/FCW/pedFCWProcess/pedFCWProcess_internal_API.h"
#include "functionality/partialRun/partialRun_API.h"

bool runPedsReporter18fps = false;

extern "C" void INIT_pedsReporters()
{
  runPedsReporter18fps = Pedestrians_API::runPedsReporter18fps();

  if (runPedsReporter18fps) {
    Pedestrians_API::initReporter(SEP::PedsReport18FPS);
  } else {
    Pedestrians_API::initReporter(SEP::PedsReport36FPS);
  }
}

extern "C" void SEP_calcPredictionForT0(int instIdx){
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!runPedsReporter18fps) {
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::kalmanAndTrack(SEP::Exp_T1,timeStampHistT0);
    Pedestrians_API::updateUpsampPedColIf(SEP::Exp_T1,timeStampHistT0);
  }
  if (CommonResources::instance().isFCWSetup()) {
    PedFCW::partialUpdatePedFCWIF(SEP::Exp_T1);
  }
}
extern "C" void SEP_fillAsilPredictionForT0(int instIdx){

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!runPedsReporter18fps) {
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::fillAsilPrediction(SEP::Exp_T1,timeStampHistT0);
  }
}

extern "C" void SEP_calcPredictionForC0(int instIdx)
{

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!runPedsReporter18fps) {
    CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
    if(CommonResources::instance().isParkingCamera(defaultCam)){
        return;
    }
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    const Prep::TimeStampHist* timeStampHistC0 = CommonResources::instance().timeStampHist(SEP::Exp_C1,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::kalmanAndTrack(SEP::Exp_C1,timeStampHistC0,timeStampHistT0);
    Pedestrians_API::updateUpsampPedColIf(SEP::Exp_C1,timeStampHistC0,timeStampHistT0);
    if (CommonResources::instance().isFCWSetup()) {
        PedFCW::partialUpdatePedFCWIF(SEP::Exp_C1);
    }
  }
}

extern "C" void SEP_fillAsilPredictionForC0(int instIdx)
{

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!runPedsReporter18fps) {
    CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
    if(CommonResources::instance().isParkingCamera(defaultCam)){
        return;
    }
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    const Prep::TimeStampHist* timeStampHistC0 = CommonResources::instance().timeStampHist(SEP::Exp_C1,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::fillAsilPrediction(SEP::Exp_C1,timeStampHistC0,timeStampHistT0);
  }
}
extern "C" void SEP_calcPredictionForT1(int instIdx)
{

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!runPedsReporter18fps) {
    CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
    if(CommonResources::instance().isParkingCamera(defaultCam)){
       return;
    }
    const Prep::TimeStampHist* timeStampHistC0 = CommonResources::instance().timeStampHist(SEP::Exp_C1,CommonResources::instance().getDefaultCameraInstance());
    const Prep::TimeStampHist* timeStampHistT1 = CommonResources::instance().timeStampHist(SEP::Exp_T2,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::kalmanAndTrack(SEP::Exp_T2,timeStampHistT1,timeStampHistC0);
    Pedestrians_API::updateUpsampPedColIf(SEP::Exp_T2,timeStampHistT1,timeStampHistC0);
    if (CommonResources::instance().isFCWSetup()) {
      PedFCW::partialUpdatePedFCWIF(SEP::Exp_T2);
    }
  }
}

extern "C" void SEP_fillAsilPredictionForT1(int instIdx)
{

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!runPedsReporter18fps) {
    CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
    if(CommonResources::instance().isParkingCamera(defaultCam)){
        return;
    }
    const Prep::TimeStampHist* timeStampHistC0 = CommonResources::instance().timeStampHist(SEP::Exp_C1,CommonResources::instance().getDefaultCameraInstance());
    const Prep::TimeStampHist* timeStampHistT1 = CommonResources::instance().timeStampHist(SEP::Exp_T2,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::fillAsilPrediction(SEP::Exp_T2,timeStampHistT1,timeStampHistC0);
  }
}

extern "C" void SEP_calcPredictionForC1(int instIdx)
{

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!runPedsReporter18fps) {
    CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
    if(CommonResources::instance().isParkingCamera(defaultCam)){
        return;
    }
    const Prep::TimeStampHist* timeStampHistT1 = CommonResources::instance().timeStampHist(SEP::Exp_T2,CommonResources::instance().getDefaultCameraInstance());
    const Prep::TimeStampHist* timeStampHistC1 = CommonResources::instance().timeStampHist(SEP::Exp_C2,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::kalmanAndTrack(SEP::Exp_C2,timeStampHistC1,timeStampHistT1);
    Pedestrians_API::updateUpsampPedColIf(SEP::Exp_C2,timeStampHistC1,timeStampHistT1);
    if (CommonResources::instance().isFCWSetup()) {
      PedFCW::partialUpdatePedFCWIF(SEP::Exp_C2);
    }
  }
}

extern "C" void SEP_fillAsilPredictionForC1(int instIdx)
{

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (!runPedsReporter18fps) {
      CameraInfo::CameraInstance defaultCam = CommonResources::instance().getDefaultCameraInstance();
      if(CommonResources::instance().isParkingCamera(defaultCam)){
          return;
      }
      const Prep::TimeStampHist* timeStampHistT1 = CommonResources::instance().timeStampHist(SEP::Exp_T2,CommonResources::instance().getDefaultCameraInstance());
      const Prep::TimeStampHist* timeStampHistC1 = CommonResources::instance().timeStampHist(SEP::Exp_C2,CommonResources::instance().getDefaultCameraInstance());
      Pedestrians_API::fillAsilPrediction(SEP::Exp_C2,timeStampHistC1,timeStampHistT1);
  }
}
extern "C" void SEP_copyPrevFrameUpsampleData(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  Pedestrians_API::copyPrevFrameUpsampPedCliques();
}

extern "C" void SEP_finalizeCliquesForNextFrame(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  Pedestrians_API::finalizeCliquesForNextFrame();
  Pedestrians::registerNumberOfCliquesEndFrame();
}

extern "C" void SEP_preReporter18fpsForT0(int instIdx)
{
  return;

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (runPedsReporter18fps) {
    Pedestrians_API::getPedReporter()->preprocessReporter(SEP::Exp_T1);
  }
}

extern "C" void SEP_calcReporter18fpsForT0(int instIdx)
{
  return;

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (runPedsReporter18fps) {
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::kalmanAndTrack(SEP::Exp_T1,timeStampHistT0);
  }
}

extern "C" void SEP_updateReporter18fpsForT0(int instIdx)
{
  return;

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (runPedsReporter18fps) {
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::updateUpsampPedColIf(SEP::Exp_T1,timeStampHistT0);
  }
}

extern "C" void SEP_fillAsilReporter18fpsForT0(int instIdx)
{
  return;

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (runPedsReporter18fps) {
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::fillAsilPrediction(SEP::Exp_T1,timeStampHistT0);
  }
}


extern "C" void SEP_preReporter18fpsForT1(int instIdx)
{
  return;

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (runPedsReporter18fps) {
    Pedestrians_API::getPedReporter()->preprocessReporter(SEP::Exp_T2);
  }
}

extern "C" void SEP_calcReporter18fpsForT1(int instIdx)
{
  return;

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (runPedsReporter18fps) {
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    const Prep::TimeStampHist* timeStampHistT1 = CommonResources::instance().timeStampHist(SEP::Exp_T2,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::kalmanAndTrack(SEP::Exp_T2,timeStampHistT1,timeStampHistT0);
  }
}

extern "C" void SEP_updateReporter18fpsForT1(int instIdx)
{
  return;

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (runPedsReporter18fps) {
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    const Prep::TimeStampHist* timeStampHistT1 = CommonResources::instance().timeStampHist(SEP::Exp_T2,CommonResources::instance().getDefaultCameraInstance());
    Pedestrians_API::copyMcpClique(SEP::Exp_T2);
    Pedestrians_API::updateUpsampPedColIf(SEP::Exp_T2,timeStampHistT1,timeStampHistT0);
  }
}

extern "C" void SEP_fillAsilReporter18fpsForT1(int instIdx)
{
  return;

  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::Pedestrians);
  if (runPedsReporter18fps) {
    const Prep::TimeStampHist* timeStampHistT0 = CommonResources::instance().timeStampHist(SEP::Exp_T1,CommonResources::instance().getDefaultCameraInstance());
    const Prep::TimeStampHist* timeStampHistT1 = CommonResources::instance().timeStampHist(SEP::Exp_T2,CommonResources::instance().getDefaultCameraInstance());

     Pedestrians_API::fillAsilPrediction(SEP::Exp_T2,timeStampHistT1,timeStampHistT0);
  }
}

