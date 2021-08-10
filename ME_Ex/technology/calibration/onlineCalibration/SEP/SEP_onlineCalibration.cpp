#include "technology/calibration/onlineCalibration/OnlineCalibration_API_internal.h"
#include "functionality/partialRun/partialRun_API.h"
#include "technology/brain2/brain2_API.h"

extern "C" void SEP_OnlineCalibration_run(int) {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::OnlineCalibration);
    OnlineCalibration_API::verifyProperties();
    OnlineCalibration_API::run();
}

extern "C" void SEP_OnlineCalibration_fillModelIF(int) {
    auto onlineCalibrationModelIF_wrapper = OnlineCalibration_API::getOnlineCalibrationModelIF_wrapper();
    PartialRun_API::updatePartialRunLoad(onlineCalibrationModelIF_wrapper, PartialRun::IFCategory::ONLINE_CALIBRATION_PR);
    
    if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::OnlineCalibration)){
        OnlineCalibration_API::fillModelIF();
    }

    PartialRun_API::updatePartialRunStore(onlineCalibrationModelIF_wrapper, PartialRun::IFCategory::ONLINE_CALIBRATION_PR);
}

extern "C" void SEP_OnlineCalibration_UpdateDriverProfileInput(int) {
    RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::OnlineCalibration);
    OnlineCalibration_API::updateDriverProfileInput(Brain2API::getDriverProfileInput());
}
