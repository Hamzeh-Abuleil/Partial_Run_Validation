#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamEgoMotionCalibrator.h"
#include "functionality/calibration/cameraInformationProperTypes.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/egomotionCovarianceMatrices.h"

namespace OnlineCalibration {
namespace Cam2Cam {

Cam2CamEgoMotionCalibrator::Cam2CamEgoMotionCalibrator(const Targets& targets) :
        ExtrinsicCalibrator(targets) {
    std::vector<Float::MEmath::Mat<3, 3, double>> covarianceMatrices(2);
    // In the future, we might have a calibrator that calibrates more than two
    // cameras at a time. For now:
    assert(targets.size() == 2);

    for (size_t i = 0; i<targets.size(); i++)
    {
        covarianceMatrices[i] = EgomotionCalibration::egomotionCovarianceMatrix(targets[i]);
    }
    _internalCalibrator.setCovErrorMatrices(covarianceMatrices[0], covarianceMatrices[1]);
}

void Cam2CamEgoMotionCalibrator::init() {
    _calibration.init(MEtl::string(coordsToStr(_calibration.getTargets()[0]) + MEtl::string("_") + coordsToStr(_calibration.getTargets()[1])).c_str()); //init properties of cam to cam calibration
}

void Cam2CamEgoMotionCalibrator::run(EmPair const& EmPair) {
    update(EmPair);
    compute();
}

const State Cam2CamEgoMotionCalibrator::getState() const
{
  return _internalCalibrator.getState();
}
const Cam2CamEMStateInfo& Cam2CamEgoMotionCalibrator::getStateInfo() const {
    return _stateInfo;
}

const Cam2CamInternalStateInfo Cam2CamEgoMotionCalibrator::getInternalStateInfo() const {
    return _internalCalibrator.getStateInfo();
}
void Cam2CamEgoMotionCalibrator::update(EmPair const& EmPair) {
    _internalCalibrator.update(EmPair.source, EmPair.target);
}

void Cam2CamEgoMotionCalibrator::compute() {
    _internalCalibrator.compute();
    _calibration.setR(_internalCalibrator.getRotation());
    _calibration.setT(_internalCalibrator.getTranslation());
    _stateInfo.setConfidence(_internalCalibrator.getConfidence());
}

void Cam2CamEgoMotionCalibrator::dumpData(ClipextIO::ClipextWriter& writer) const {
#ifdef DEBUG
    ExtrinsicCalibrator::dumpData(writer);
    _stateInfo.dumpData(writer);
#endif
}

}
}
