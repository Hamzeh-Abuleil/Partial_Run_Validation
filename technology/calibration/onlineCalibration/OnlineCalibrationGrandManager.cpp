#include "technology/calibration/onlineCalibration/OnlineCalibrationGrandManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/CalibratorManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamEgoMotionManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/Cam2WorldManager.h"
#include "functionality/partialRun/partialRun_API.h"
#include "utils/CRC/common/crcComputation.h"
#include <assert.h>

namespace OnlineCalibration {

OnlineCalibrationGrandManager* OnlineCalibrationGrandManager::_instance = nullptr;

void OnlineCalibrationGrandManager::init() {
    assert(_instance == nullptr);
    _instance = new OnlineCalibrationGrandManager();

    auto onlineCalibrationModelIF_wrapper = _instance->getOnlineCalibrationModelIF_wrapper();
    if (onlineCalibrationModelIF_wrapper != nullptr){
      PartialRun_API::registerIF(onlineCalibrationModelIF_wrapper);
    }
}

OnlineCalibrationGrandManager* OnlineCalibrationGrandManager::instance() {
    assert(_instance != nullptr);
    return _instance;
}

OnlineCalibrationGrandManager::~OnlineCalibrationGrandManager() {
    delete _instance;
}

OnlineCalibrationGrandManager::OnlineCalibrationGrandManager()  : _OcIF(), _OnlineCalibrationModelIF_wrapper(&_OcIF){

    for (ExManager::iterator iter = _managersStorage.exBegin(); iter != _managersStorage.exEnd(); iter++) {
        if (*iter) {
            (*iter)->init();
        }
    }
    for (InManager::iterator iter = _managersStorage.inBegin(); iter != _managersStorage.inEnd(); iter++) {
        if (*iter) {
            (*iter)->init();
        }
    }
}

void OnlineCalibrationGrandManager::run() {

    for (ExManager::iterator iter = _managersStorage.exBegin(); iter != _managersStorage.exEnd(); iter++) {
        runHelper(iter);
    }
    for (InManager::iterator iter = _managersStorage.inBegin(); iter != _managersStorage.inEnd(); iter++) {
        if (*iter) {
            runHelper(iter);
        }
    }
}

template <class Iter>
void OnlineCalibrationGrandManager::runHelper(Iter iter) {
    // only in GV not PC: Start GSF (each has different dependencies)
    assert(*iter != nullptr);
    RunConfig runConfig = (*iter)->shouldRun();
    if (runConfig != RunConfig::DONT_RUN) {
        bool ok = (*iter)->prepSources();
        if (ok)
        {
            (*iter)->run();
        }
    }
    (*iter)->calcResult(); // updates state also if run() doesn't happen
}

const ExtrinsicCalibResults& OnlineCalibrationGrandManager::getCurrentCalib(CoordSys source, CoordSys target) const {
    return _managersStorage.getCurrentCalib(source, target);
}

const IntrinsicCalibResults& OnlineCalibrationGrandManager::getCurrentCalib(CoordSys cam) const {
    return _managersStorage.getCurrentCalib(cam);
}

const MEtypes::ptr_vector<MEtypes::RealCamInstance>& OnlineCalibrationGrandManager::getEmCameras() {
    return _managersStorage.getEmCameras();
}


void OnlineCalibrationGrandManager::dumpData() {
#ifdef DEBUG
    static ClipextIO::ClipextWriter writer(".onlineCalibration");
    ClipextIO::ExpID expId = ClipextIO::CEXT_SLOW;
    writer.setExpID(expId);

    for(ExManager::iterator iter = _managersStorage.exBegin(); iter != _managersStorage.exEnd(); iter++) {
        (*iter)->dumpData(writer);
    }
    for(InManager::iterator iter = _managersStorage.inBegin(); iter != _managersStorage.inEnd(); iter++) {
        if (*iter) {
            (*iter)->dumpData(writer);
        }
    }

    writer.flushTuple();
#endif
}

void OnlineCalibrationGrandManager::fillModelIF() {

    for(ExManager::iterator iter = _managersStorage.exBegin(); iter != _managersStorage.exEnd(); iter++) {
        (*iter)->fillModelIF(_OcIF);
    }
    for(InManager::iterator iter = _managersStorage.inBegin(); iter != _managersStorage.inEnd(); iter++) {
        if (*iter) {
            (*iter)->fillModelIF(_OcIF);
        }
    }
    _OcIF.editable().ASIL_CRC = utilsCrcCommon::crcComputation::crc16((const unsigned char*)(&(_OcIF->data)), sizeof(OnlineCalibrationModelIFData));
    _OcIF.update();
}

void OnlineCalibrationGrandManager::updateDriverProfileInput(dstruct_t* driverProfile) {
    for(ExManager::iterator iter = _managersStorage.exBegin(); iter != _managersStorage.exEnd(); iter++) {
        (*iter)->updateDriverProfileInput(driverProfile);
    }
}

bool OnlineCalibrationGrandManager::verifyProperties() {
    bool verified = true;
    for(ExManager::iterator iter = _managersStorage.exBegin(); iter != _managersStorage.exEnd(); iter++) {
        if (*iter) {
            if (!(*iter)->verifyProperties())
                verified = false;
        }
    }
    for(InManager::iterator iter = _managersStorage.inBegin(); iter != _managersStorage.inEnd(); iter++) {
        if (*iter) {
            if (!(*iter)->verifyProperties())
                verified = false;
        }
    }
    return verified;
}
}
