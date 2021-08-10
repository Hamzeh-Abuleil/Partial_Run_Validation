#include "technology/calibration/onlineCalibration/OnlineCalibration_API_internal.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationGrandManager.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"

#include "utilities/cameraInformation/cameraInformation_API.h"

namespace OnlineCalibration_API {
using namespace OnlineCalibration;

void init() {
  OnlineCalibrationGrandManager::init();
}

void run() {
  OnlineCalibrationGrandManager::instance()->run();
}

void fillModelIF() {
    OnlineCalibrationGrandManager::instance()->fillModelIF();
}

void updateDriverProfileInput(dstruct_t* driverProfile) {
    OnlineCalibrationGrandManager::instance()->updateDriverProfileInput(driverProfile);
}

Fix::MEimage::Sync<OnlineCalibrationModelIF>* getOnlineCalibIF() {
    return OnlineCalibrationGrandManager::instance()->getOnlineCalibIF();
}

PartialRun::OnlineCalibrationModelIF_wrapper* getOnlineCalibrationModelIF_wrapper() {
    return OnlineCalibrationGrandManager::instance()->getOnlineCalibrationModelIF_wrapper();
}

EOnlineCalibrationModelIF* getEOnlineCalibIF() {
    return OnlineCalibrationGrandManager::instance()->getEOnlineCalibIF();
}

const ExtrinsicCalibResults& getCurrentCalib(CoordSys source,
                                             CoordSys target) {
  assert(source != target);
  return OnlineCalibrationGrandManager::instance()->getCurrentCalib(source, target);
}

const IntrinsicCalibResults& getCurrentCalib(CoordSys cam) {
  assert(CameraInfo::exists(coordsToInstance(cam)));
  return OnlineCalibrationGrandManager::instance()->getCurrentCalib(cam);
}

const MEtypes::ptr_vector<MEtypes::RealCamInstance>& getEmCameras() {
  return OnlineCalibrationGrandManager::instance()->getEmCameras();
}

bool verifyProperties() {
  return OnlineCalibrationGrandManager::instance()->verifyProperties();
}

}
