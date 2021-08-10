#include "technology/MTF/MTF_API.h"
#include "technology/include/SEP/SEP_InstIdx.h"


extern "C"
{
  void SEP_runMTF(int instIdx)
  {
    MTF_API::runMTF();
  }
}
