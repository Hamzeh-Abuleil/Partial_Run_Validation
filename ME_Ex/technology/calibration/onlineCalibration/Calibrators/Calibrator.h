#ifndef __OC_CALIBRATOR_H
#define __OC_CALIBRATOR_H

#include "technology/calibration/onlineCalibration/Calibrators/StateInfo.h"

namespace OnlineCalibration {

class Calibrator {
public:
    virtual ~Calibrator() {}
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const = 0;
};

}

#endif
