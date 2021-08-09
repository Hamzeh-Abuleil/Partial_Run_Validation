#include "technology/DS/common/SLIinterfaceAPI.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/DS/SLIReporter/SLIReporter_API.h"
#include "technology/brain2/brain2_API.h"
#include "technology/DS/SLIutils/common/SLIUtils.h"

extern "C" void INIT_SLIReporter(int instIdx) {
  SLIReporter_API::initializeModule(SLIinterfaceAPI::getSLI_IF());
}
