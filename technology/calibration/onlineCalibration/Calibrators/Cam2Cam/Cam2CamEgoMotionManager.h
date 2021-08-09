#ifndef __OC_CAM2CAM_EM_MANAGER_H_
#define __OC_CAM2CAM_EM_MANAGER_H_

#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibratorManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamEgoMotionSources.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamEgoMotionCalibrator.h"

namespace OnlineCalibration {
namespace Cam2Cam {

class Cam2CamEgoMotionManager : public ExtrinsicCalibratorManager {
public:
    static TargetsLists& getTargetLists();
    static TargetsLists& setTargetLists();
    Cam2CamEgoMotionManager(Targets targets);
    virtual void init() override;
    bool prepSources() override;
    RunConfig shouldRun() override;
    void run() override;
    void calcResult(void) override;
    State calcState(int &degradeCause) override;
    void dumpData(ClipextIO::ClipextWriter& writer) const override;
    virtual void fillModelIF(Fix::MEimage::Sync<OnlineCalibrationModelIF> &OcModelIF) override;
    virtual bool verifyProperties() override;
private:
    static TargetsLists targetsList;
    Cam2CamEgoMotionCalibrator _calibrator;
    Cam2CamEgoMotionSources _sources;
};

}
}

#endif //__OC_CAM2CAM_EM_MANAGER_H_
