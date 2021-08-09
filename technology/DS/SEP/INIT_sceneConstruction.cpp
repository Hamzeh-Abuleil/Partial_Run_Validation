/*
 * INIT_sceneConstruction.cpp
 *
 */
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/DS/sceneConstruction/sceneConstruction_API.h"
#include "technology/brain2/brain2_API.h"
#include "functionality/partialRun/partialRun_API.h"
#include "functionality/partialRun/wrappers/ESceneUnderstandingIF_wrapper.h"
#include <memory>

extern "C" void INIT_sceneConstruction(int instIdx) {
  CHECK_FORWARD_CAM;
  SceneUnderstanding::SceneConstruction::API::initializeModule();

  PartialRun_API::registerIF(new PartialRun::ESceneUnderstandingIF_wrapper(SceneUnderstanding::SceneConstruction::API::getESceneUnderstandingIF()));
  Brain2API::setSceneUnderstandingIF(SceneUnderstanding::SceneConstruction::API::getSceneUnderstandingIF(), SceneUnderstanding::SceneConstruction::API::getESceneUnderstandingIF());

}
