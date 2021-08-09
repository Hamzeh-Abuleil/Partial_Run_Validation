#include "technology/calibration/WMBC/wmbc_ServicesUser.h"
WmbcServicesUser::WmbcServicesUser(){}
void WmbcServicesUser::init(){
  ServiceLocator_API::ServiceLocator::instance()->request(WorldModel::EgoMotion::EgoMotionService_API::name(), this);
  ServiceLocator_API::ServiceLocator::instance()->request(WorldModel::RM::RoadModelService_API::name(), this);
  ServiceLocator_API::ServiceLocator::instance()->request(CameraToCameraAPI::CameraToCameraService_API::name(), this);
}

void WmbcServicesUser::patch(const std::string &serviceName, ServiceLocator_API::APIProvider *service)
{
  if(serviceName==WorldModel::EgoMotion::EgoMotionService_API::name()){
    get<WorldModel::EgoMotion::EgoMotionService_API>() = static_cast<WorldModel::EgoMotion::EgoMotionService_API*>(service);
  }
  else if(serviceName==WorldModel::RM::RoadModelService_API::name()){
    get<WorldModel::RM::RoadModelService_API>() = static_cast<WorldModel::RM::RoadModelService_API*>(service);
  }
  else if(serviceName==CameraToCameraAPI::CameraToCameraService_API::name()){
    get<CameraToCameraAPI::CameraToCameraService_API>() = static_cast<CameraToCameraAPI::CameraToCameraService_API*>(service);
  }
}