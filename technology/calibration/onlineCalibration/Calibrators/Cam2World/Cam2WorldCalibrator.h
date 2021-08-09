#ifndef __OC_CAM2WORLD_CALIBRATOR_H_
#define __OC_CAM2WORLD_CALIBRATOR_H_

#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibrator.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/Cam2WorldSources.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/steadyStateCalibrator.h"

namespace OnlineCalibration {
namespace Cam2World {

class Cam2WorldCalibrator : public ExtrinsicCalibrator {
public:
    Cam2WorldCalibrator(const Targets& targets);
    // void init() override;
    void init() {};
    void init(const Cam2WorldSources& source);
    void run(Cam2WorldSources& source);
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override;
    void stopSPC() { _steadyStateCalibrator.stopSPC();}
    void restartSPC() { _steadyStateCalibrator.restartSPC();}

    int getDegradeCause() { return _steadyStateCalibrator.degradeCause();}
    int getEmState() { return _steadyStateCalibrator.getEmState(); }
    int getRmState() { return _steadyStateCalibrator.getRmState(); }
    const Cam2WorldBaseLineProperties* getBaseLineProperites() const { return _baseLineProperties; }
    const Cam2WorldStateInfo& getStateInfo() const {  return _stateInfo; }
    virtual bool verifyProperties() override;

private:
    void compute();
    void update(const Cam2WorldSources& source);
    Cam2WorldProperties* _properties;
    Cam2WorldBaseLineProperties* _baseLineProperties;
    Cam2WorldStateInfo _stateInfo;
    SteadyStateCalibrator _steadyStateCalibrator;
};

}
}

#endif
