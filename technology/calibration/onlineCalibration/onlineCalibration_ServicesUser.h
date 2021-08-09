#ifndef _ONLINE_CALIBRATION_USER_
#define _ONLINE_CALIBRATION_USER_
#include "technology/worldModel/egoMotionService_API.h"
#include "technology/worldModel/roadModelService_API.h"
struct OnlineCalibrationServicesUser: public ServiceLocator_API::userOf<WorldModel::EgoMotion::EgoMotionService_API,
                                                                        WorldModel::RM::RoadModelService_API>{
  
  void patch(const std::string& serviceName,ServiceLocator_API::APIProvider* service) override;
  void init();
  static OnlineCalibrationServicesUser& instance(){
    static OnlineCalibrationServicesUser single;
    return single;
  }
  virtual ~OnlineCalibrationServicesUser()=default;
  private:
    OnlineCalibrationServicesUser();
};
#endif