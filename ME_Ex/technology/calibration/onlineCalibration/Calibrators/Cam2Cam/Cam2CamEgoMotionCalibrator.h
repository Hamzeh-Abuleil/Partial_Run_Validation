#ifndef __OC_CAM2CAM_EM_CALIBRATOR_H_
#define __OC_CAM2CAM_EM_CALIBRATOR_H_

#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibrator.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamEgoMotionSources.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "egomotionCalibrator.h"

namespace OnlineCalibration {
namespace Cam2Cam {

class Cam2CamEgoMotionCalibrator : public ExtrinsicCalibrator {
public:
    Cam2CamEgoMotionCalibrator(const Targets& targets);
    void init() override;
    void run(EmPair const& EmPair);
    const Cam2CamEMStateInfo& getStateInfo() const;
    const Cam2CamInternalStateInfo getInternalStateInfo() const;
    const State getState() const;
    virtual void dumpData(ClipextIO::ClipextWriter &writer) const override;

  private:
    void compute();
    void update(EmPair const& EmPair);
    EgomotionCalibration::EgomotionCalibrator _internalCalibrator;
    Cam2CamEMStateInfo _stateInfo;
};
}
}

#endif
