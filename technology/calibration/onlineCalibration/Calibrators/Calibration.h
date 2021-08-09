#ifndef __OC_CALIBRATION_H_
#define __OC_CALIBRATION_H_

#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"

namespace OnlineCalibration {

class Calibration {
public:
    Calibration(const Targets& targets) : _targets(targets) {}
    const Targets& getTargets() const { return _targets; }
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
        writer.setData("targets", reinterpret_cast<const int *>(&_targets[0]), _targets.size());
#endif
    }
private:
    Targets _targets;
};

}

#endif
