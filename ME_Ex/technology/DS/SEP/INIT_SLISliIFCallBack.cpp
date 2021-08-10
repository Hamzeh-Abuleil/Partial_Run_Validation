#include "technology/brain2/brain2_API.h"
#include "technology/DS/common/SLIinterfaceAPI.h"
#include "utilities/realWorld/realWorld_API.h"

extern "C" void INIT_SLISliIFCallBack(int instIdx)
{
  // if countryCode is USA or CANADA then the sli model of brain2 will be filled in INIT_SLIUSAattention
  if (RealWorld::regionCode() == RealWorld::USA_RC || RealWorld::regionCode() == RealWorld::CANADA_RC)
    return;
  
  Brain2API::setSLIIF(SLIinterfaceAPI::getSLI_IF(), SLIinterfaceAPI::getESLI_IF());
  
}
