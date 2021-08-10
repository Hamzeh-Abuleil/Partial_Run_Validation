#ifndef __OC_CALIBRATION_RESULTS_H_
#define __OC_CALIBRATION_RESULTS_H_

#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/mobilib/fix/common/MEXmisc/failSafeDefs.h"

namespace OnlineCalibration {

class CalibrationResults {
public:
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
        writer.setData("state", reinterpret_cast<const int *>(&state));
        writer.setData("FailSafe", reinterpret_cast<const int *>(&fs));
#endif
    }

    State state = State::NUM_OF_STATES;
    int stateDegradeCause = (int)StateDegradeCause::NO_DEGRADE;
    MEfailsafe::FailSafeMode fs;
};

}

#endif //__OC_CALIBRATION_RESULTS_H_
