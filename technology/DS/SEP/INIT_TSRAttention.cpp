#include "technology/brain2/prepSys/common/prepSys_API_Internal.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/brain2/prepSys/common/imageCollection.h"
#include "technology/brain2/brain2_API.h"
#include "functionality/calibration/clipProperties.h"
#include "basicTypes/image/image.h"
#include "technology/DS/TSRAttention/TSRAttention_API.h"


extern "C" void INIT_TSRAttention(int instIdx)
{
  TSR::Attention_API::initializeModule();
}

