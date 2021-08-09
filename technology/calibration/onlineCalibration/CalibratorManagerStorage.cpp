#include "technology/calibration/onlineCalibration/CalibratorManagerStorage.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamEgoMotionManager.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/Cam2WorldManager.h"

namespace OnlineCalibration {

CalibratorManagerStorage::CalibratorManagerStorage() {

  const TargetsLists& c2c_targetslists = Cam2Cam::Cam2CamEgoMotionManager::setTargetLists();
  const TargetsLists& c2w_targetslists = Cam2World::Cam2WorldManager::setTargetLists();
  _extrinsicManagers.reserve(sizeof(c2c_targetslists) + sizeof(c2w_targetslists));

  //Came2CamEM
  for (Targets targets : c2c_targetslists) {
        _extrinsicManagers.emplace_back(std::make_shared<Cam2Cam::Cam2CamEgoMotionManager>(targets));
        updateMap(targets);
  }

  //Cam2World
  for (Targets targets : c2w_targetslists) {
          _extrinsicManagers.emplace_back(std::make_shared<Cam2World::Cam2WorldManager>(targets));
          updateMap(targets);
  }

  // map
  for(auto array : _mapExManagers) {
    array.fill(nullptr);
  }

  //intrinsic
  _intrinsicManagers.fill(nullptr);

//  const TargetsLists& in_targetslists = intrinsic::intrinsicManager::getTargetLists();
//  for (Targets targets : in_targetslists) {
//    assert(targets.size()==0);
//    _intrinsicManager[targets[0]] = std::move(std::make_shared<IntrinsicManager>(list));
//  }
}


void CalibratorManagerStorage::updateMap(Targets& targets) {
  for(Targets::iterator i = targets.begin(); i != targets.end() - 1; i++) {
    for(Targets::iterator j = i + 1; j != targets.end(); j++) {
      _mapExManagers[*i][*j] = _extrinsicManagers.back();
      _mapExManagers[*j][*i] = _extrinsicManagers.back();
    }
  }
}


const ExtrinsicCalibResults& CalibratorManagerStorage::getCurrentCalib(CoordSys source, CoordSys target) const {
  assert(source != CoordSys::NUM_OF_CAMS && source != CoordSys::NUM_OF_COORDS && target != CoordSys::NUM_OF_CAMS && target != CoordSys::NUM_OF_COORDS);
  assert(_mapExManagers[source][target]!=nullptr);
  return _mapExManagers[source][target]->getCurrentCalibration();
}

const IntrinsicCalibResults& CalibratorManagerStorage::getCurrentCalib(CoordSys cam) const {
  assert(_intrinsicManagers[cam]!=nullptr);
  return _intrinsicManagers[cam]->getCurrentCalibration();
}

const MEtypes::ptr_vector<MEtypes::RealCamInstance>& CalibratorManagerStorage::getEmCameras() {
  static MEtypes::ptr_vector<MEtypes::RealCamInstance> emCams(CoordSys::NUM_OF_CAMS);
  for (int i=0; i < NUM_OF_CAMS; i++) {
    for (int j=0; j < CoordSys::NUM_OF_COORDS; j++) {
      if (_mapExManagers[i][j] != nullptr) {
        emCams.emplace_back(MEtypes::CamInstToRCI(coordsToInstance((CoordSys)i)));
        break;
      }
    }
  }
  return emCams;
}

}
