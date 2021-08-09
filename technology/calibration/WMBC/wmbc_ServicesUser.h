#ifndef _WMBC_USER_
#define _WMBC_USER_
#include "technology/worldModel/egoMotionService_API.h"
#include "technology/worldModel/roadModelService_API.h"
#include "technology/calibration/cameraToCamera/cameraToCameraService_API.h"
struct WmbcServicesUser:public ServiceLocator_API::userOf<WorldModel::EgoMotion::EgoMotionService_API,
                                                          WorldModel::RM::RoadModelService_API,
                                                          CameraToCameraAPI::CameraToCameraService_API>{
  
  void patch(const std::string& serviceName,ServiceLocator_API::APIProvider* service) override;
  void init();
  static WmbcServicesUser& instance(){
    static WmbcServicesUser single;
    return single;
  }
  virtual ~WmbcServicesUser()=default;
  private:
    WmbcServicesUser();
};
#endif