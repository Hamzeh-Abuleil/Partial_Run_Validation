#include "utilities/TAC/TAC_API.h"
#include "technology/brain2/brain2_API.h"

extern "C" void INIT_TAC(int instIdx)
{
  TACUtil_API::initTAC();

  Brain2API::setTACIF(TACUtil_API::getTACIF());
}
