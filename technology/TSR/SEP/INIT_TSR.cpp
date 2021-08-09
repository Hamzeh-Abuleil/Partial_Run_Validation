#include "technology/TSR/TSRinterface_API.h"
#include "technology/brain2/brain2_API.h"

extern "C" void INIT_TSR(int instIdx) {
  TSRinterfaceAPI::initTSR();
  
  Brain2API::setTSRIF(TSRinterfaceAPI::getTSRIF());
}
 


