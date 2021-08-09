#include "technology/hlb/HLB_API.h"
#include "technology/hlb/hlbConfig.h"
#include "../common/HLBComm.h"

extern "C" {
void SEP_HLBprepareBrightSceneHistogramTasks(int instIdx) 
{
  HLB_API::prepareBrightSceneHistogramTasks();
}

void SEP_HLBcalculateBrightSceneScore(int instIdx)
{
  HLB_API::calculateBrightSceneScore();
}

void SEP_HLBLitNightDecision(int instIdx)
{
  HLB_API::litNightDecision();
}

void SEP_HLBfindStreetLights(int instIdx)
{
  HLB_API::findStreetLights();
}

void SEP_HLBoncomingOverGuardrail(int instIdx)
{
  HLB_API::oncomingOverGuardrail();
}

void SEP_HLBRegularityDetector(int instIdx)
{
  HLB_API::regularityDetector();
}
  
void SEP_HLBOscillatingTrafficLightsDetector(int instIdx)
{
  HLB_API::oscillatingTrafficLightsDetector();
}

void SEP_HLBOCMotionClassification(int instIdx)
{
  HLB_API::ocMotionClassification();
}

void SEP_HLBRedTrafficLightsDetector(int instIdx)
{
  HLB_API::redTrafficLightsDetector();
}
  
void SEP_HLBcheckRectangleSign(int instIdx) {
  HLB_API::checkRectangleSign();
}

void SEP_HLBupdateExplicitDay(int instIdx)
{
  HLB_API::updateExplicitDay();
}

void SEP_HLBupdateLowSpeedAndHighYawDecisions(int instIdx) 
{
  HLB_API::lowSpeedAndHighYawDecisions();
  //PATCH!! NEVER REMOVE THIS GOAL FROM DAY AGENDA. 
  SEP_HLBupdateExplicitDay(instIdx);
}

void SEP_HLBupdateClustersAfterTracking(int)
{
  HLB_API::updateClustersAfterTracking();
}

void SEP_HLBsendDisplayData(int instIdx) {
  HLBComm::sendDisplayDataToEyeQClient();
}

void SEP_HLBprepDarkestIntensityHistForCoupling(int instIdx) 
{
  HLB_API::prepDarkestIntensityHistForCoupling();
}

void SEP_HLBinitFrame(int instIdx)
{
  HLBConfig::initFrame(); // new config
  HLBConfiguration::initFrame(); // old config
  HLB_API::initHLBFrame();
}

void SEP_HLBAddVDTLApprovals(int instIdx)
{
  HLB_API::addVDTLApprovals();
}

void SEP_HLBfinalApproval(int instIdx)
{
  HLB_API::finalApproval();
}
  
void SEP_HLBfinalHLBDecision(int instIdx)
{
  HLB_API::finalHLBDecision();
}

void SEP_HLBRecordRoadInfo(int instIdx)
{
  HLB_API::recordRoadInfo();
}

void SEP_HLBcreateCoupleCandidates(int instIdx) {
  HLB_API::createCoupleCandidates();
}

void SEP_HLBcreateTLSuspects(int instIdx) {
  HLB_API::createTLSuspects();
}

void SEP_HLBcreateLightCouples(int instIdx) {
  HLB_API::createLightCouples();
}

void SEP_HLBcreateApproved(int instIdx)
{
  if (!NightExposuresInfo::validFrame()) {
    return;
  }
  HLB_API::createApproved(HLB_API::graceCounterInMiliSecs(), HLB_API::numberOfSLHeights());
}

void SEP_HLBspotPositionDistanceEstimation(int instIdx) {
  HLB_API::spotPositionDistanceEstimation();
}

void SEP_HLBspotPositionGradualEstimation(int instIdx) {
  HLB_API::spotPositionGradualEstimation();
}

void SEP_HLBcreateTruckApproved(int instIdx) {
  HLB_API::createTruckApproved();
}

void SEP_HLBupdateSpotFeaturesScores(int instIdx) {
  HLB_API::updateSpotFeaturesScores();
}

void SEP_HLBprepareFOGHistogramTasks(int instIdx)
{
  HLB_API::prepareFogHistogramTasks();
}

void SEP_HLBDecideFog(int instIdx)
{
  HLB_API::fogDecision();
}
  
void SEP_HLBDecideFlkScene(int instIdx)
{
  HLB_API::flkDecision();
}

void SEP_HLBprepareCouplingClassifier(int instIdx)
{
  HLB_API::prepareCouplingClassifier();
}

void SEP_HLBretrieveCouplingClassifier(int instIdx)
{
  HLB_API::retrieveCouplingClassifier();
  HLB_API::prepareCouplingClassifierSmall();
}
void SEP_HLBretrieveCouplingClassifierSmall(int instIdx)
{
  HLB_API::retrieveCouplingClassifierSmall();
}

void SEP_HLBprepareOncomingVsReflectorClassifierSmall(int instIdx)
{
  HLB_API::prepareOncomingVsReflectorClassifierSmall();
}
void SEP_HLBretrieveOncomingVsReflectorClassifierSmall(int instIdx)
{
  HLB_API::retrieveOncomingVsReflectorClassifierSmall();
}
void SEP_HLBretrieveOncomingVsReflectorClassifierMedium(int instIdx)
{
  HLB_API::retrieveOncomingVsReflectorClassifierMedium();
}
void SEP_HLBretrieveOncomingVsReflectorClassifierLarge(int instIdx)
{
  HLB_API::retrieveOncomingVsReflectorClassifierLarge();
}

}
