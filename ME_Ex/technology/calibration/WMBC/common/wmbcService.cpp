#include "technology/calibration/WMBC/common/wmbcService.h"
#include "technology/calibration/WMBC/common/wmbc_API_internal.h"
Fix::MEimage::Sync<WmbcIF>* WmbcService::getWmbcIF(){
  return WMBC::getWmbcIF();
}
EWmbcIF* WmbcService::getEWmbcIF(){
  return WMBC::getEWmbcIF();
}
Fix::MEimage::Sync<WmbcIF>* WmbcService::getQuickWmbcIF(){
  return WMBC::getQuickWmbcIF();
}
bool WmbcService::isRunning(){
  return WMBC::isRunning();
}
bool WmbcService::isRunningAutofix(){
  return WMBC::isRunningAutofix();
}
bool WmbcService::isRunningSPC(){
  return WMBC::isRunningSPC();
}
bool WmbcService::isRunningSLC(){
  return WMBC::isRunningSLC();
}
float WmbcService::confidence(){
  return WMBC::confidence();
}
int WmbcService::confidenceGrade(){
  return WMBC::confidenceGrade();
}
void WmbcService::getFoeInExpAllocated(CameraInfo::CameraInstance cinst, PrepSys::exp_mask exp, float& x, float& y, bool interim/*=false*/){
  WMBC::getFoeInExpAllocated(cinst,exp,x,y, interim);
}
int  WmbcService::autoFix_yawDelta(CameraInfo::CameraInstance inst){
  return WMBC::autoFix_yawDelta(inst);
}
int WmbcService::autoFix_horizonDelta(CameraInfo::CameraInstance inst){
  return WMBC::autoFix_horizonDelta(inst);
}
bool WmbcService::yawConverged(CameraInfo::CameraInstance inst){
  return WMBC::yawConverged(inst);
}
bool WmbcService::horizonConverged(CameraInfo::CameraInstance inst){
  return WMBC::horizonConverged(inst);
}
Fix::MEimage::Sync<MultiCameraAutoFixIF>* WmbcService::getAutoFixIF(){
  return WMBC::getAutoFixIF();
}
EMultiCameraAutoFixIF* WmbcService::getEAutoFixIF(){
  return WMBC::getEAutoFixIF();
}
