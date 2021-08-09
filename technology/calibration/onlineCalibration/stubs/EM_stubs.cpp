#include "technology/worldModel/egoMotion/Manager/EmManager.h"

namespace WorldModel{
namespace EgoMotion {

  MEtypes::ptr_vector<EmData>* getUmVisionMeasurements(const int exposure) {

    MEtypes::ptr_vector<WorldModel::EgoMotion::EmData>* EmDatas = new MEtypes::ptr_vector<EmData> (MAX_NUM_OF_SYSTEM_VISION_SOURCES);
    EmDatas->push_back(
            EmData(
              MEtypes::EfficientMatrix<double, 4, 4>::one(),
              MEtypes::Vector<double, 3>(),
              globalTick(),
              globalTick(),
              0,
              0,
              globalTick(),
              MEtypes::CoordinateSystem::CoS_FORWARD,
              1,
              MEtypes::Vector<float,  6>()
            )
        );
    EmDatas->push_back(
            EmData(
              MEtypes::EfficientMatrix<double, 4, 4>::one(),
              MEtypes::Vector<double, 3>(),
              globalTick(),
              globalTick(),
              0,
              0,
              globalTick(),
              MEtypes::CoordinateSystem::CoS_REAR,
              1,
              MEtypes::Vector<float,  6>()
            )
        );
    EmDatas->push_back(
            EmData(
              MEtypes::EfficientMatrix<double, 4, 4>::one(),
              MEtypes::Vector<double, 3>(),
              globalTick(),
              globalTick(),
              0,
              0,
              globalTick(),
              MEtypes::CoordinateSystem::CoS_REAR_CORNER_LEFT,
              1,
              MEtypes::Vector<float,  6>()
            )
        );
    EmDatas->push_back(
            EmData(
              MEtypes::EfficientMatrix<double, 4, 4>::one(),
              MEtypes::Vector<double, 3>(),
              globalTick(),
              globalTick(),
              0,
              0,
              globalTick(),
              MEtypes::CoordinateSystem::CoS_REAR_CORNER_RIGHT,
              1,
              MEtypes::Vector<float,  6>()
            )
        );
    return EmDatas; 
  }

}
}
