#include "technology/calibration/onlineCalibration/OnlineCalibration_API_internal.h"
#include "technology/brain2/brain2_API.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationService.h"
#include "technology/calibration/onlineCalibration/onlineCalibration_ServicesUser.h"

extern "C" void INIT_OnlineCalibration_init(int){
  OnlineCalibrationServicesUser::instance().init();
  OnlineCalibration_API::init();
  OnlineCalibration_API::verifyProperties();
  Brain2API::setOnlineCalibrationIF(OnlineCalibration_API::getOnlineCalibIF(), OnlineCalibration_API::getEOnlineCalibIF());
  OnlineCalibrationService *ocService=new OnlineCalibrationService();
  ServiceLocator_API::ServiceLocator::instance()->advertise(ocService);
}
