#include "technology/calibration/onlineCalibration/onlineCalibration_ServicesUser.h"
OnlineCalibrationServicesUser::OnlineCalibrationServicesUser(){}
void OnlineCalibrationServicesUser::init(){
  ServiceLocator_API::ServiceLocator::instance()->request(WorldModel::EgoMotion::EgoMotionService_API::name(), this);
  ServiceLocator_API::ServiceLocator::instance()->request(WorldModel::RM::RoadModelService_API::name(), this);
}

void OnlineCalibrationServicesUser::patch(const std::string &serviceName, ServiceLocator_API::APIProvider *service)
{
  if(serviceName==WorldModel::EgoMotion::EgoMotionService_API::name()){
    get<WorldModel::EgoMotion::EgoMotionService_API>() = static_cast<WorldModel::EgoMotion::EgoMotionService_API*>(service);
    return;
  }
  if(serviceName==WorldModel::RM::RoadModelService_API::name()){
    get<WorldModel::RM::RoadModelService_API>() = static_cast<WorldModel::RM::RoadModelService_API*>(service);
  }
}