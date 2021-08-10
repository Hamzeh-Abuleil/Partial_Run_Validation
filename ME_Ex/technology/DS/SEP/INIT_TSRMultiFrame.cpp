#include "technology/DS/TSRMultiFrame/tsrMF_API.h"
#include "technology/DS/SLIutils/common/TSRClipextManager.h"
#include "technology/DS/TSRMultiFrame/i386/tsrMFDebuggerDefs.h"

extern "C" void INIT_TSRMultiFrame(int instIdx)
{
  TSR::API::initializeModule();
  DEBUG_CODE(TSR::ClipextManager::instance().init(false);)
}

