#ifndef __OC_CAM2wORLD_H_
#define __OC_CAM2wORLD_H_

#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibratorManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/Cam2WorldSources.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/Cam2WorldCalibrator.h"

namespace OnlineCalibration {
namespace Cam2World {

class Cam2WorldManager : public ExtrinsicCalibratorManager {
public:
    static TargetsLists& getTargetLists();
    static TargetsLists& setTargetLists();
    Cam2WorldManager(Targets targets);
    virtual void init() override;
    void prepSourcesInit();
    bool prepSources() override;
    RunConfig shouldRun() override;
    void run() override;
    void calcResult(void) override;
    State calcState(int &degradeCause) override;
    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override;
    virtual void fillModelIF(Fix::MEimage::Sync<OnlineCalibrationModelIF> &OcModelIF) override;
    virtual void updateDriverProfileInput(dstruct_t* driverProfile) override;
    virtual bool verifyProperties() override;
private:
    State calcStateSpc(int &degradeCause);
    void calcSpcProgress();
    void computeSpcStatusAndError();
    void updateSPCTrigger(bool spcTrigger);
    void stateMachineClassic(State &state);
    void stateMachineSimpleThresholds(State &state);
    void stateMachineThresholdWMemory(State &state);
    void stateMachineTempName(State &state);
    static TargetsLists targetsList;
    Cam2WorldCalibrator _calibrator;
    SpcData _spcData;
    Cam2WorldSources _sources;
    State _prevState;
    State _innerState;
    State _prevInnerState;
    ConfLevel _confLevel;
    ConfLevel _prevConfLevel;
    bool _wasHighConf;
    unsigned int _confidence;
    unsigned int _stringentConfidence;
    unsigned int _stateTimeCounter;
    unsigned int _prevStateTimeCounter;
    unsigned int _confLevelTimeCounter;
    unsigned int _highConfParams[4];   // hysteresis: min th, max th, increment, stringentHighTh
    unsigned int _highConfTh; // threshold for high confidence
    unsigned int _lowConfTh; // threshold for low confidence
    unsigned int _thresholdsDelta; // threshold for low confidence
    unsigned int _stringentHighThRng[2];
    unsigned int _highThRng[2];
    unsigned int _stateMachine;
    unsigned int _startDelayLength;
    unsigned int _suspectTimeTh;
    bool  _frameProgress;
};

}
}

#endif
