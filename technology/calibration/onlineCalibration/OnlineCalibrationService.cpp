#include "technology/calibration/onlineCalibration/OnlineCalibrationService.h"
#include "technology/calibration/onlineCalibration/OnlineCalibration_API_internal.h"
void OnlineCalibrationService::init(){
  OnlineCalibration_API::init();
}
void OnlineCalibrationService::run(){
  OnlineCalibration_API::run();
}
void OnlineCalibrationService::fillModelIF(){
  OnlineCalibration_API::fillModelIF();
}
Fix::MEimage::Sync<OnlineCalibration::OnlineCalibrationModelIF> *OnlineCalibrationService::getOnlineCalibIF(){
  return OnlineCalibration_API::getOnlineCalibIF();
}
PartialRun::OnlineCalibrationModelIF_wrapper *OnlineCalibrationService::getOnlineCalibrationModelIF_wrapper(){
  return OnlineCalibration_API::getOnlineCalibrationModelIF_wrapper();
}
OnlineCalibration::EOnlineCalibrationModelIF *OnlineCalibrationService::getEOnlineCalibIF(){
  return OnlineCalibration_API::getEOnlineCalibIF();
}
const OnlineCalibration::ExtrinsicCalibResults &OnlineCalibrationService::getCurrentCalib(OnlineCalibration::CoordSys source, OnlineCalibration::CoordSys target){
  return OnlineCalibration_API::getCurrentCalib(source,target);
}
const OnlineCalibration::IntrinsicCalibResults &OnlineCalibrationService::getCurrentCalib(OnlineCalibration::CoordSys cam){
  return OnlineCalibration_API::getCurrentCalib(cam);
}
const MEtypes::ptr_vector<MEtypes::RealCamInstance>& OnlineCalibrationService::getEmCameras(){
  return OnlineCalibration_API::getEmCameras();
}