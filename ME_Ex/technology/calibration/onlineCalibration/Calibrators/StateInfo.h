#ifndef __OC_STATE_INFO_H
#define __OC_STATE_INFO_H

#include "utilities/clipextIO/clipextIO.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wData.h"

namespace OnlineCalibration {

class StateInfo {
public:
    StateInfo() : _confidence(0), _std(0) {}
    double getConfidence() const { return _confidence;}
    unsigned int getStd() const { return _std;}
    void setConfidence(double confidence) { _confidence = confidence;}
    void setStd(unsigned int std) { _std = std;}
    void dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
        writer.setData("Confidence", &_confidence);
        writer.setData("Std", &_std);
#endif
    }
private:
    double _confidence;
    unsigned int _std;
};

class Cam2CamEMStateInfo : public StateInfo {};

class Cam2WorldStateInfo : public StateInfo {
public:
    Cam2WorldStateInfo() : StateInfo(), _stringentConfidence(0), _inRange(false), _spcData() {}
    unsigned int getStringentConfidence() const { return _stringentConfidence;}
    bool getInRange() const { return _inRange;}
    bool getStableSig() const { return _stableSig;}
    const Cam2World::SpcData& getSpcData() const { return _spcData;}
    void setStringentConfidence(unsigned int confidence) { _stringentConfidence = confidence;}
    void setInRange(bool inRange) { _inRange = inRange;}
    void setStableSig(bool stableSig) { _stableSig = stableSig;}
    void setSpcData(const Cam2World::SpcData spcData) { _spcData = spcData;}
private:
    unsigned int _stringentConfidence;
    bool _inRange;
    bool _stableSig;
    Cam2World::SpcData _spcData;
};

}

#endif //__OC_STATE_INFO_H
