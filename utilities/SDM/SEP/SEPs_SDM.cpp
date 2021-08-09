#include "utilities/SDM/safetyDiagnosticManager_API.h"

extern "C" void SEP_SDMupdateFrame(int instIdx)
{
  SDM::updateFrame(); 

}
