#include "technology/brain2/brain2_API.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "functionality/calibration/clipProperties.h"
#include "functionality/interface/applicationData.h"
#include "utilities/linesRegistry/linesRegistry_API.h"
#include "technology/include/SEP/SEP_InstIdx.h"
#include "technology/brain2/brainDefaults.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/pedestrians/pedestrians/common/Pedestrians_API_Internal.h"
#include "technology/pedestrians/pedestrians/common/pedKalmanFilter_API.h"
#include "technology/pedestrians/pedestrians/common/bicycleWheelsDetectionSF.h"
#include "technology/calibration/utilities/cameraProjections/cameraProjectionsAPI.h"
#include "technology/calibration/utilities/cameraProjections/common/initRectifications.h"
#include "functionality/ServiceLocator/ServiceLocator_API.h"
#include "technology/pedestrians/pedestrians/common/PedestriansService.h"
#include "technology/pedestrians/pedestrians/cliquer/common/cliquerService.h"


#define WARP_TYPE RectificationAPI::WARP_MAIN

namespace RVCImageWarp {
  bool fullyInitialized(){
    return (CameraProj::isWarpExist(WARP_TYPE, CameraInfo::e_FORWARD))?true:false;
  }

  void pinholeArgesRect(Float::MEgeo::Rect &rect){
    CameraProj::targetToSourceArgesRect(WARP_TYPE, rect, CameraInfo::e_FORWARD);
  }
}


namespace Fix
{
  namespace MEimage
  {
    class Rect;
  }
}

// prototypes:
Pedestrians_API::ProjectType getProjectType();
extern "C" void  INIT_pedAsilSafetyMonitor(int instIdx){
  Pedestrians_API::initAsilSafeMonitor();
}
extern "C" void INIT_pedKalmanPool(int instIdx){
  PEDKalman::initPedKalmanPool(instIdx);
}

extern "C" void INIT_pedKalmanPoolUpsample(int instIdx){
//  PEDKalman::initPedKalmanPoolUpsample();
//#ifndef EYEQ_HW_IMPL
//  PEDKalman::initPedKalmanPoolPredictHist();
//#endif
}

extern "C" void INIT_pedestriansCommonResources(int instIdx) {

  
  Pedestrians_API::ProjectType project = getProjectType();

  Pedestrians_API::initPedsProperties();
  Pedestrians_API::initSurroundCameras();
  
  Pedestrians_API::initPedResources(Brain2API::getDModel(),
                                    SEP::CROSSINGSIGNALCLIQUESTRACKING,
                                    SEP::CROSSINGSIGNALSFTRACKING,
                                    SEP::CROSSINGSIGNALCLIQUESTRACKING_N1,
                                    SEP::CROSSINGSIGNALSFTRACKING_N1,
                                    project,
                                    iTracker::iTrackerInstance::CROSSINGSIGNALCLIQUESTRACKING);
  PedestriansService *pedService=new PedestriansService();
  ServiceLocator_API::ServiceLocator::instance()->advertise(pedService);
  Pedestrians::CliquerService * clqService=new Pedestrians::CliquerService();
  ServiceLocator_API::ServiceLocator::instance()->advertise(clqService);
}


extern "C" void INIT_pedestrians(int instIdx)
{

  //2. init Peds Flages
  int pedDebug = 0;
  int testSomething = 0;
  bool noPeds = false;
  bool noPedsT2 = false;

  bool noBikes;
  bool noCrossingBicycles = false;
  if(SEP::PedsWithoutBikes == instIdx){
    noBikes = true;
  } else {
    noBikes = false;
  }

  // bool noVCDetection = true;

  if (Pedestrians_API::MAGNA_RVC == getProjectType()) {
    noBikes = true;
    //    noVCDetection = false;
  }

  bool stereoEnable = BrainDefaults::brainDefaults().stereoEnable;

  std::string perfectName = (Debug::Args::instance().getStickyValue("-sPedPerfect", "")).c_str();

#ifdef MEwin
  if (Debug::Args::instance().existsParameter("-sPedDebug"))
    pedDebug = 1;
  else if (Debug::Args::instance().existsParameter("-sPedDebugNoWin"))
    pedDebug = 2;
  else if (Debug::Args::instance().existsParameter("-sPedDebugQueriesDraw"))
    pedDebug = 3;
  else if (Debug::Args::instance().existsParameter("-sPedDebugQueriesPrint"))
    pedDebug = 4;
#endif

  if (Pedestrians_API::noBikesFromCalib()){ //from default calib/command line
    std::cerr<<"Disabling Bikes detection"<<std::endl;
    noBikes = true;
  }
  if (Pedestrians_API::noPedsT2FromCalib() || (Debug::Args::instance().existsParameter("-snoPedsT2"))){
    noPedsT2 = true;
  }
  if (Pedestrians_API::noCrossingBicyclesFromCalib()){ //from default calib/command line
    //std::cerr<<"Disabling Crossing bicycles detection"<<std::endl;
    noCrossingBicycles = true;
  }

  if ((Debug::Args::instance().existsParameter("-snoPeds")) ||
      (!BrainDefaults::brainDefaults().detectPeds)){
      noPeds=true;
      std::cerr<<"Disabling Peds detection"<<std::endl;
    }

  Pedestrians::PedOptionalInputs pedOptionalInputs;
  pedOptionalInputs.NVEnabled   = (Brain2API::pedsRunModes() == Brain2Properties::e_PRM_NV    || Brain2API::pedsRunModes() == Brain2Properties::e_PRM_BOTH);
  pedOptionalInputs.mainEnabled = (Brain2API::pedsRunModes() == Brain2Properties::e_PRM_MAIN  || Brain2API::pedsRunModes() == Brain2Properties::e_PRM_BOTH);
  Pedestrians_API::initPedFlags( pedDebug, testSomething, noPeds, noPedsT2,noBikes, noCrossingBicycles, stereoEnable, pedOptionalInputs);

  //3. init Peds
  const ClipProperties* clipProperties = Brain2API::getClipProperties();
  Fix::MEimage::Coordinate foe(CommonResources::instance().getYaw(), CommonResources::instance().getHorizon());

  std::string session(clipProperties->clipSession().c_str());
  std::string::size_type loc = session.find("/mobileye/images/");
  if (loc != std::string::npos){
    session = session.substr(17);
  }
  loc = session.find("/mobileye/");
  if (loc != std::string::npos){
    session = session.substr(10);
  }

  Pedestrians_API::init(Fix::MEimage::Size(), perfectName,
                        clipProperties->clipName().c_str(), session.c_str(),
                        clipProperties->firstFrame(), clipProperties->lastFrame(), foe,
                        SEP::PEDPARTSTRACKER, SEP::PEDGOTRACKER);

  Pedestrians_API::setModelIF(Brain2API::getModelIF());
  Brain2API::setPedCollectionIF(Pedestrians_API::getPedCollectionIF(), Pedestrians_API::getEPedCollectionIF());

  if (Pedestrians_API::MAGNA_RVC == getProjectType()) {
    Pedestrians_API::setSpeedAneDistanceImageWarp(&RVCImageWarp::pinholeArgesRect);
  }
}


Pedestrians_API::ProjectType getProjectType()
{

  Pedestrians_API::ProjectType projectType = Pedestrians_API::DELPHI;

  switch (Brain2API::applicationData().customer) {
  case(AFTERMARKET_VER):
  case(DELPHI_VER):
  case(KAFASL7_VER):
  case(STEREO_VER):
  case(HIGH_RES):
    projectType = Pedestrians_API::DELPHI;
    break;
  case(TAYLOR_VER):
    projectType = Pedestrians_API::TAYLOR;
    break;
  case(REARCAM_VER):
    {
      if ( Fix::MEstd::MAGNA == CameraInfo::cameraCustomer() || Fix::MEstd::MAGNA_PED == CameraInfo::cameraCustomer())
        projectType = Pedestrians_API::MAGNA_RVC;
      else
        projectType = Pedestrians_API::BMW_RVC;
    }
    break;
  default:
    projectType = Pedestrians_API::DELPHI;
    assert(false);
  }
  return projectType;
}


extern "C" void SEP_setLinesRegistrySupMapEffective(int instIdx)
{
  assert(false);
}
