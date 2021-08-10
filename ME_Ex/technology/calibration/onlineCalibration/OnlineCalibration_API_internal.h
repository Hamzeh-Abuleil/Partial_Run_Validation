#ifndef __ONLINE_CALIBRATION_API_H_
#define __ONLINE_CALIBRATION_API_H_

#include <memory>
#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/ExtrinsicCalibrationResults.h"
#include "technology/calibration/onlineCalibration/IntrinsicCalibrationResults.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationModelIF.h"
#include "functionality/partialRun/wrappers/OnlineCalibrationModelIF_wrapper.h"

namespace OnlineCalibration_API {


void init();
void run();
void fillModelIF();
void updateDriverProfileInput(dstruct_t* driverProfile);
Fix::MEimage::Sync<OnlineCalibration::OnlineCalibrationModelIF>* getOnlineCalibIF();
PartialRun::OnlineCalibrationModelIF_wrapper* getOnlineCalibrationModelIF_wrapper();
OnlineCalibration::EOnlineCalibrationModelIF* getEOnlineCalibIF();
const OnlineCalibration::ExtrinsicCalibResults& getCurrentCalib(
                                            OnlineCalibration::CoordSys source,
                                            OnlineCalibration::CoordSys target);
const OnlineCalibration::IntrinsicCalibResults& getCurrentCalib(
                                            OnlineCalibration::CoordSys cam);
const MEtypes::ptr_vector<MEtypes::RealCamInstance>& getEmCameras();

bool verifyProperties();

}

#endif
