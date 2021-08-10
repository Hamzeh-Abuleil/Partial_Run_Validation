#include "technology/hlb/HLB_API.h"
#include "technology/hlb/hlbConfig.h"
#include "technology/nightSpots/spotsManager_API.h"
#include "technology/brain2/brain2_API.h"
#include "technology/hlb/common/HLBMemory.h"
#include "technology/hlb/common/spotPosition.h"
#include "technology/hlb/common/gradualInLoadBalance.h"
#include "technology/hlb/common/HLBVehiclesBuilder.h"
#include "technology/hlb/common/HLBProperties.h"
#include "technology/nightSpots/nightExposuresInfo/nightExposuresConfig.h"
#include "functionality/interface/applicationData.h"

static bool initHLB = true;
static bool first = true;

extern "C" void INIT_HLB(int)
{

	// this is a patch to avoid initializing the HLB system when running in SLI Only mode. we should find
	// a better way to do this
	if (first){
		first = false;
		if (!NightExposuresInfo::validExposures() || Debug::Args::instance().existsParameter("-sSLIOnly")){
			initHLB = false;
			return;
		}
	}
	if (!initHLB){
		return;
	}

  ApplicationMode::applicationMode_t appMode = Brain2API::applicationData().getMode();
  if (appMode == ApplicationMode::SPTAC) {
    return;
  }

  HLBConfig::init(); // new config
  HLBConfiguration::init(); // old config
  HLB_API::init(HLBConfig::get());
  SpotsManager_API::enableClient(SpotsManager_API::HLB, !HLB_API::noHLB());
  int tlWidthInCm = (HLBMemory::instance().properties())->STFLA_AVERAGE_TL_WIDTH();
  int ocWidthInCm = (HLBMemory::instance().properties())->STFLA_AVERAGE_HL_WIDTH();
  bool hlbOnly = Debug::Args::instance().existsParameter("-sHLBOnly");
  HLBMemory::instance().spotPosition()->init(Brain2API::getModelIF(),tlWidthInCm,ocWidthInCm,hlbOnly,
                                             *HLBMemory::instance().properties()); // using VD's approved for angle/distance estimation
  HLBMemory::instance().gradualInLoadBalance()->init(Brain2API::getModelIF(), hlbOnly, *HLBMemory::instance().properties()); // using VD's approved for angle/distance estimation
  
  HLBMemory::instance().vehiclesBuilder()->init(Brain2API::getModelIF(),hlbOnly); 
}
 

