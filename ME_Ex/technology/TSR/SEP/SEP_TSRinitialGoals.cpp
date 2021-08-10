#include "technology/include/SEP/SEP_InstIdx.h"
#include "technology/TSR/TSRinterface_API.h"
using namespace SEP;
using std::cerr;
using std::cout;
using std::endl;

extern "C" void SEP_initTSRFrame(int instIdx) 
{
  TSRinterfaceAPI::initTSRFrame();
}
extern "C" void SEP_combineAndManage(int instIdx) 
{
  TSRinterfaceAPI::combineAndManage();
}
extern "C" void SEP_clearFrame(int instIdx) 
{
  TSRinterfaceAPI::clearFrame();
}

extern "C" void SEP_endOfTSRFrame(int instIdx) 
{
  TSRinterfaceAPI::endOfTSRFrame();
}

extern "C" void SEP_prepareRectsForClassifyTSR(int instIdx)
{
  TSRinterfaceAPI::prepareRectsForClassifyTSR();
}

extern "C" void SEP_postClassifierGetResults(int instIdx)
{
  TSRinterfaceAPI::postClassifierGetResults();
}

extern "C" void SEP_manageMFObjects(int instIdx)
{
  TSRinterfaceAPI::manageMF();
}
extern "C" void SEP_prepareRectsForTrack(int instIdx)
{
  TSRinterfaceAPI::prepareRectsForTrack();
}
extern "C" void SEP_postRectsForTrack(int instIdx)
{
  
  TSRinterfaceAPI::postTracking();
}




