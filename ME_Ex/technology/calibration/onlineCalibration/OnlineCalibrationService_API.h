#ifndef __ONLINE_CALIBRATION_SERVICE_API_H_
#define __ONLINE_CALIBRATION_SERVICE_API_H_
#include "functionality/ServiceLocator/ServiceLocator_API.h"
#include "functionality/partialRun/wrappers/OnlineCalibrationModelIF_wrapper.h"
#include "basicTypes/geo/GeometryDefinitions.h"
#include "basicTypes/containers/ptr_vector.h"

namespace Fix{
  namespace MEimage{
    template<class T>
    class Sync;
  }
}
namespace OnlineCalibration{
  struct OnlineCalibrationModelIF;
  class ExtrinsicCalibResults;
  class IntrinsicCalibResults;
}
namespace OnlineCalibration_API {
  class OnlineCalibrationService_API: public ServiceLocator_API::APIProvider{
    public:
      virtual ~OnlineCalibrationService_API()=default;
      virtual void init()=0;
      virtual void run()=0;
      virtual void fillModelIF()=0;
      virtual Fix::MEimage::Sync<OnlineCalibration::OnlineCalibrationModelIF>* getOnlineCalibIF()=0;
      virtual PartialRun::OnlineCalibrationModelIF_wrapper* getOnlineCalibrationModelIF_wrapper()=0;
      virtual OnlineCalibration::EOnlineCalibrationModelIF* getEOnlineCalibIF()=0;
      virtual const OnlineCalibration::ExtrinsicCalibResults& getCurrentCalib(
                                            OnlineCalibration::CoordSys source,
                                            OnlineCalibration::CoordSys target)=0;
      virtual const OnlineCalibration::IntrinsicCalibResults& getCurrentCalib(
                                      OnlineCalibration::CoordSys cam)=0;
      virtual const MEtypes::ptr_vector<MEtypes::RealCamInstance>& getEmCameras()=0;
      static std::string name(){return "OnlineCalibration";}
  };
}
#endif