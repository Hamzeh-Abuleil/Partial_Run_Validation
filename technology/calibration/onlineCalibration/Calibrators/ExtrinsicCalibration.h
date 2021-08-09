#ifndef __OC_EXTRINSIC_CALIBRATION_H_
#define __OC_EXTRINSIC_CALIBRATION_H_

#include "technology/calibration/onlineCalibration/Calibrators/Calibration.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibrationProperties.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wData.h"

namespace OnlineCalibration {

class ExtrinsicCalibration : public Calibration {
public:
    ExtrinsicCalibration() : ExtrinsicCalibration(Targets()) { }
    ExtrinsicCalibration(const Targets& targets) :
            Calibration(targets),
            _R(CalibUtils::id3f()),
            _T(CalibUtils::zVec3f()) {
        _properties = new CalibrationProperties();
        _baselineProperties = new CalibrationBaselineProperties();
    }
    virtual ~ExtrinsicCalibration() {}

    const float* const getR() const {
        return _R.begin()->begin();
    }
    void setR(Float::MEmath::Mat<3, 3, float> const& R) {
        _R = R;
    }
    void setR(Float::MEmath::Mat<3, 3, double> const& R) {
        _R = CalibUtils::matd2f(R);
    }
    const float* const getT() const {
        return _T.begin();
    }
    void setT(Float::MEmath::Vec<3, float> const& t) {
        _T = t;
    }
    void setT(Float::MEmath::Vec<3, double> const& t) {
        _T = CalibUtils::vecd2f(t);
    }
    const Cam2World::ResultBook& getComponents() const {
      return _components;
    }
    void setComponents(Cam2World::ResultBook const& c) {
      _components = c;
    }
    void init(const char* section) {
        _properties->init(section);
        _baselineProperties->init(section);
        _R = _properties->getR();
        _T = _properties->getT();
    }
    bool verifyProperties() {
        return _properties->verifyAllProps() and _baselineProperties->verifyAllProps();
    }

    virtual void dumpData(ClipextIO::ClipextWriter& writer) const override {
#ifdef DEBUG
        Calibration::dumpData(writer);
        _properties->dumpData(writer);
        _baselineProperties->dumpData(writer);
#endif
    }
private:
    CalibrationProperties* _properties;
    CalibrationBaselineProperties* _baselineProperties;
    Cam2World::ResultBook _components;
    Float::MEmath::Mat<3,3,float> _R;
    Float::MEmath::Vec<3,float> _T;
};

}

#endif //__OC_EXTRINSIC_CALIBRATION_H_
