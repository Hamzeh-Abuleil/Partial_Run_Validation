#include "technology/generalObjects/common/goblin_API_internal.h"
#include "technology/generalObjects/common/main.h"
#include "technology/brain2/brain2_API.h"

#include "functionality/partialRun/partialRun_API.h"


extern "C" void INIT_GOBLINDetection_InitClip(int instIdx)
{
  CHECK_FORWARD_CAM;
  if(Brain2API::runGoblin()){
    GOBLIN::Goblin_Main::instance().initClip();
  }

}

extern "C" void SEP_GOBLINDetection_RunFrame(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::goblin);

  CHECK_FORWARD_CAM;
  if(Brain2API::runGoblin()){
    GOBLIN::Goblin_Main::instance().runFrame();
  }
}
