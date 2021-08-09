#ifndef __OC_CALIBRATOR_MANAGER_STORAGE_H_
#define __OC_CALIBRATOR_MANAGER_STORAGE_H_

#include <memory>

#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/Calibrators/CalibratorManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/IntrinsicCalibration.h"
#include "technology/calibration/onlineCalibration/Calibrators/IntrinsicCalibratorManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibration.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibratorManager.h"

namespace OnlineCalibration {

using ExManager = std::vector<std::shared_ptr<ExtrinsicCalibratorManager>>;
using InManager = std::array<std::shared_ptr<IntrinsicCalibratorManager>,  CoordSys::NUM_OF_CAMS>;

class CalibratorManagerStorage {
public:
  CalibratorManagerStorage();

  ExManager::iterator exBegin() {return _extrinsicManagers.begin();}
  ExManager::iterator exEnd() {return _extrinsicManagers.end();}
  InManager::iterator inBegin() {return _intrinsicManagers.begin();}
  InManager::iterator inEnd() {return _intrinsicManagers.end();}

  const ExtrinsicCalibResults& getCurrentCalib(CoordSys source, CoordSys target) const;
  const IntrinsicCalibResults& getCurrentCalib(CoordSys cam) const;
  const MEtypes::ptr_vector<MEtypes::RealCamInstance>& getEmCameras();

private:
  ExManager _extrinsicManagers;
  InManager _intrinsicManagers;
  std::array<std::array<std::shared_ptr<ExtrinsicCalibratorManager>,
                        CoordSys::NUM_OF_COORDS>, CoordSys::NUM_OF_COORDS> _mapExManagers; //is a map to which extrinsic manager to use for each quarry
  void updateMap(Targets& targets);


};


}

#endif //__OC_CALIBRATOR_MANAGER_STORAGE_H_
