#include "technology/brain2/brain2_API.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/DS/roadMarkings/SLIRoadMarkings_API.h"
#include "technology/DS/common/SLIinterfaceMemory.h"
#include "technology/DS/common/SLIProperties.h"

extern "C" void INIT_SLIRoadMarkings(int instIdx) {
  CHECK_FORWARD_CAM;
  RoadMarkings::API::initializeModule();
}


