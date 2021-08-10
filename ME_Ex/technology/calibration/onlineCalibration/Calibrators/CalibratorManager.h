#ifndef __OC_CALIBRATION_MANAGER_H
#define __OC_CALIBRATION_MANAGER_H

#include "technology/calibration/onlineCalibration/CalibrationResults.h"
#include "technology/calibration/onlineCalibration/Calibrators/Calibrator.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationModelIF.h"

namespace OnlineCalibration {

class CalibratorManager {
public:
    CalibratorManager(Targets targets); //_calibrator is created in derived class
    virtual ~CalibratorManager() {
    }
    virtual void init() = 0;
    virtual bool prepSources() = 0;
    virtual RunConfig shouldRun() = 0; //checks with stateInfo last GFI and sets _runConfig
    virtual void run() = 0;

    /*
     * for each particular manager -
     * call Intrinsic/Extrinsic Calibrator Manager :: calcResultHelper()
     */
    virtual void calcResult(void) = 0;

    /*
     * for each particular manager -
     * call Intrinsic/Extrinsic Calibrator Manager :: calcStateHelper() or implement calcState
     */
    virtual State calcState(int &degradeCause) = 0;

    virtual void dumpData(ClipextIO::ClipextWriter& writer) const;

    virtual void fillModelIF(Fix::MEimage::Sync<OnlineCalibrationModelIF> &OcModelIF) = 0;

    virtual bool verifyProperties() = 0;

private:
    RunConfig _runConfig = RunConfig::INVALID;

protected:
    Targets _targets;
    unsigned int _lastRunGFI = -1;
    virtual State calcStateHelper(const StateInfo& stateInfo, const unsigned int lastRunGFI) const;
};

}

#endif
