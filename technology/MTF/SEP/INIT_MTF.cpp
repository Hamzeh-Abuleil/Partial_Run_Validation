#include "technology/MTF/MTF_API.h"
#include "functionality/calibration/propertiesManager.h"
#include "technology/brain2/brain2_API.h"


extern "C" void INIT_MTF(int instIdx)
{
  MTF_API::initMTF();

  Brain2API::setMTFIF(MTF_API::getMTFIF(), MTF_API::getEMTFIF());
}


extern "C" void INIT_commandLineMTF(int instIdx)
{
  MTF_API::initCommandLineMTF(getCalibrationManager());
}

