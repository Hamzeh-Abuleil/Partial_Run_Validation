#ifndef __ONLINE_CALIBRATION_GRAND_MANAGER_H_
#define __ONLINE_CALIBRATION_GRAND_MANAGER_H_

#include "technology/calibration/onlineCalibration/CalibrationResults.h"
#include "technology/calibration/onlineCalibration/CalibratorManagerStorage.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationModelIF.h"
#include "technology/mobilib/fix/common/MEXimage/sync.h"
#include "functionality/partialRun/wrappers/OnlineCalibrationModelIF_wrapper.h"

namespace OnlineCalibration {

class OnlineCalibrationGrandManager {
public:
    static void init();
    static OnlineCalibrationGrandManager* instance();
    virtual ~OnlineCalibrationGrandManager();

    void run();
    const ExtrinsicCalibResults& getCurrentCalib(CoordSys source, CoordSys target) const;
    const IntrinsicCalibResults& getCurrentCalib(CoordSys cam) const;

    const MEtypes::ptr_vector<MEtypes::RealCamInstance>& getEmCameras();

    void dumpData();
    void fillModelIF();
    void updateDriverProfileInput(dstruct_t* driverProfile);

    Fix::MEimage::Sync<OnlineCalibrationModelIF>* getOnlineCalibIF() {
        return &_OcIF;
    }
    PartialRun::OnlineCalibrationModelIF_wrapper* getOnlineCalibrationModelIF_wrapper() {
        return &_OnlineCalibrationModelIF_wrapper;
    }
    EOnlineCalibrationModelIF* getEOnlineCalibIF() {
        return &_eOcIF;
    }

    bool verifyProperties();

private:
    OnlineCalibrationGrandManager();

    template<class Iter>
    void runHelper(Iter iter);

    static OnlineCalibrationGrandManager* _instance;
    CalibratorManagerStorage _managersStorage;

    Fix::MEimage::Sync<OnlineCalibrationModelIF> _OcIF;
    PartialRun::OnlineCalibrationModelIF_wrapper _OnlineCalibrationModelIF_wrapper;
    EOnlineCalibrationModelIF _eOcIF;
};

}

#endif //__ONLINE_CALIBRATION_GRAND_MANAGER_H_
