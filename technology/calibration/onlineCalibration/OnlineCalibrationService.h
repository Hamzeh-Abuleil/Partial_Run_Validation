#ifndef _ONLINE_CALIBRATION_SERVICE_H_
#define _ONLINE_CALIBRATION_SERVICE_H_
#include "technology/calibration/onlineCalibration/OnlineCalibrationService_API.h"
class OnlineCalibrationService: public OnlineCalibration_API::OnlineCalibrationService_API{
  public:
    ~OnlineCalibrationService()=default;
    void init()  override;
    void run()  override;
    void fillModelIF()  override;
    Fix::MEimage::Sync<OnlineCalibration::OnlineCalibrationModelIF>* getOnlineCalibIF()  override;
    PartialRun::OnlineCalibrationModelIF_wrapper* getOnlineCalibrationModelIF_wrapper()  override;
    OnlineCalibration::EOnlineCalibrationModelIF* getEOnlineCalibIF()  override;
    const OnlineCalibration::ExtrinsicCalibResults& getCurrentCalib(
                                            OnlineCalibration::CoordSys source,
                                            OnlineCalibration::CoordSys target)  override;
    const OnlineCalibration::IntrinsicCalibResults& getCurrentCalib(
                                      OnlineCalibration::CoordSys cam)  override;
    const MEtypes::ptr_vector<MEtypes::RealCamInstance>& getEmCameras() override;
};
#endif