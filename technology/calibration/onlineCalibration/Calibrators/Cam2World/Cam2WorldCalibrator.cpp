#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/Cam2WorldCalibrator.h"
//#include "technology/calibration/onlineCalibration/ExtrinsicCalibrationResults.h"
#include "technology/calibration/onlineCalibration/OnlineCalibration_API_internal.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wUtils.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wOutputIF.h"

namespace OnlineCalibration {
namespace Cam2World {

Cam2WorldCalibrator::Cam2WorldCalibrator(const Targets& targets) : ExtrinsicCalibrator(targets){
    _properties = new Cam2WorldProperties();
    _baseLineProperties = new Cam2WorldBaseLineProperties();
}

void Cam2WorldCalibrator::init(const Cam2WorldSources& source) {
    char sec[256];
    sprintf(sec, "[%s]", coordsToStr(_calibration.getTargets()[1]));
    _calibration.init(sec); //init properties of camera calibrated to world (targets[0]=WORLD)
    _properties->init(sec); //init properties of camera calibrated to world (targets[0]=WORLD)
    _baseLineProperties->init(sec); //init properties of camera calibrated to world (targets[0]=WORLD)

    // When yaw, pitch, are given in the new properties this can be revived, currently they still come via yawFull, horizonFull, rollAngle
    // Moreover it is better to have consistency between R and the rest of quantities in modelIF
    // Float::MEmath::Mat<3, 3, float> R = composeRotationMatrixFromSensorAngles(_properties->_Yaw(), _properties->_Pitch(), _properties->_Roll());
    // _calibration.setR(R);
    _steadyStateCalibrator.setExtraParams(source);
    _steadyStateCalibrator.setCameraData(source);
    _steadyStateCalibrator.computeInit(source);
    _calibration.setR(_steadyStateCalibrator.rotation());
    _calibration.setT(_steadyStateCalibrator.translation());
    _calibration.setComponents(_steadyStateCalibrator.results());
}

void Cam2WorldCalibrator::run(Cam2WorldSources& source) {
  update(source);
  compute();
}

void Cam2WorldCalibrator::update(const Cam2WorldSources& source) {
  CoordSys tgt = _calibration.getTargets().back();
  if (tgt != CoordSys::FORWARD) {
    ASSERT(0);
    return;
  }
  _steadyStateCalibrator.update(source);
}

void Cam2WorldCalibrator::compute() {
  CoordSys tgt = _calibration.getTargets().back();
  if (tgt != CoordSys::FORWARD) {
    ASSERT(0);
    return;
  }

  _steadyStateCalibrator.compute();
  _calibration.setR(_steadyStateCalibrator.rotation());
  _calibration.setT(_steadyStateCalibrator.translation());
  _calibration.setComponents(_steadyStateCalibrator.results());
  _stateInfo.setConfidence(_steadyStateCalibrator.confidence());
  _stateInfo.setStringentConfidence(_steadyStateCalibrator.stringentConfidence());
  _stateInfo.setInRange(_steadyStateCalibrator.isCalibInRange());
  _stateInfo.setStableSig(_steadyStateCalibrator.isSignalStable());
  _stateInfo.setSpcData(_steadyStateCalibrator.spcData());
  Cam2WorldOutputIF::instance().tostdoutRT(e_RESULT, e_GREEN, "[C2WCalibrator::compute] ",
                                           Float::MEmath::Mat<3,3,float>(_calibration.getR()),
                                           Float::MEmath::Vec<3,float>(_calibration.getT()));
}

void Cam2WorldCalibrator::dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
    ExtrinsicCalibrator::dumpData(writer);
    _properties->dumpData(writer);
    _baseLineProperties->dumpData(writer);;
    _stateInfo.dumpData(writer);
#endif
}

bool Cam2WorldCalibrator::verifyProperties() {
    return _properties->verifyAllProps() and
           _baseLineProperties->verifyAllProps() and
            ExtrinsicCalibrator::verifyProperties();
}

}
}
