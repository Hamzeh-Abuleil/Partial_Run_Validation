#include "technology/pedestrians/animals/Animals_API.h"
#include "technology/pedestrians/animals/common/animalsEnums.h"
#include "technology/pedestrians/animals/common/animalsHelpers.h"
#include "technology/mobilib/fix/common/MEXmisc/timeStampUtils.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/include/SEP/SEP_InstIdx.h"

// ---------- Predict animals colection IF -----------------

extern "C" void SEP_calcAnimalsPredictionForT0(int instIdx)
{
  // if (Animals_API::animalsCurrLCNight())
  //   return;

  const Prep::TimeStampHist* timeStampHistT0 = PrepSys_API::getTimeStamp(PrepSys::exp_mask::T0);
  
  //Animals_API::correctRectForCurrentExp(SEP::Exp_T1, timeStampHistT0);
  Animals_API::correctRectForCurrentExp(Animals::EXP_T0, timeStampHistT0);
  
  const Prep::TimeStampHist* timeStampHistT0finish = PrepSys_API::getTimeStamp(PrepSys::exp_mask::T0);

  //Animals_API::updateUpsampAnimalsColIf(SEP::Exp_T1, timeStampHistT0);
  Animals_API::updateUpsampAnimalsColIf(Animals::EXP_T0, timeStampHistT0finish);
}

extern "C" void SEP_calcAnimalsPredictionForC0(int instIdx)
{
  // if (Animals_API::animalsCurrLCNight())
  //   return;

  const Prep::TimeStampHist* timeStampHistT0 = PrepSys_API::getTimeStamp(PrepSys::exp_mask::T0);
  const Prep::TimeStampHist* timeStampHistC0 = PrepSys_API::getTimeStamp(PrepSys::exp_mask::C0);
  
  // Animals_API::correctRectForCurrentExp(SEP::Exp_C1, timeStampHistC0, timeStampHistT0);
  Animals_API::correctRectForCurrentExp(Animals::EXP_C0, timeStampHistC0, timeStampHistT0);

  const Prep::TimeStampHist* timeStampHistT0Finish = PrepSys_API::getTimeStamp(PrepSys::exp_mask::T0);
  const Prep::TimeStampHist* timeStampHistC0Finish = PrepSys_API::getTimeStamp(PrepSys::exp_mask::C0);

  //Animals_API::updateUpsampAnimalsColIf(SEP::Exp_C1, timeStampHistC0, timeStampHistT0);
  Animals_API::updateUpsampAnimalsColIf(Animals::EXP_C0, timeStampHistC0Finish, timeStampHistT0Finish);
}


extern "C" void SEP_calcAnimalsPredictionForT1(int instIdx)
{
  // if (Animals_API::animalsCurrLCNight())
  //   return;

  const Prep::TimeStampHist* timeStampHistC0 = PrepSys_API::getTimeStamp(PrepSys::exp_mask::C0);
  const Prep::TimeStampHist* timeStampHistT1 = PrepSys_API::getTimeStamp(PrepSys::exp_mask::T1);
  
  //Animals_API::correctRectForCurrentExp(SEP::Exp_T2, timeStampHistT1, timeStampHistC0);
  Animals_API::correctRectForCurrentExp(Animals::EXP_T1, timeStampHistT1, timeStampHistC0);

  const Prep::TimeStampHist* timeStampHistC0Finish = PrepSys_API::getTimeStamp(PrepSys::exp_mask::C0);
  const Prep::TimeStampHist* timeStampHistT1Finish = PrepSys_API::getTimeStamp(PrepSys::exp_mask::T1);

  // Animals_API::updateUpsampAnimalsColIf(SEP::Exp_T2, timeStampHistT1, timeStampHistC0);
  Animals_API::updateUpsampAnimalsColIf(Animals::EXP_T1, timeStampHistT1Finish, timeStampHistC0Finish);
}

extern "C" void SEP_calcAnimalsPredictionForC1(int instIdx)
{
  // if (Animals_API::animalsCurrLCNight())
  //   return;

  const Prep::TimeStampHist* timeStampHistT1 = PrepSys_API::getTimeStamp(PrepSys::exp_mask::T1);
  const Prep::TimeStampHist* timeStampHistC1 = PrepSys_API::getTimeStamp(PrepSys::exp_mask::C1);
  
  // Animals_API::correctRectForCurrentExp(SEP::Exp_C2, timeStampHistC1, timeStampHistT1);
  Animals_API::correctRectForCurrentExp(Animals::EXP_C1, timeStampHistC1, timeStampHistT1);

  const Prep::TimeStampHist* timeStampHistT1finish = PrepSys_API::getTimeStamp(PrepSys::exp_mask::T1);
  const Prep::TimeStampHist* timeStampHistC1finish = PrepSys_API::getTimeStamp(PrepSys::exp_mask::C1);

  // Animals_API::updateUpsampAnimalsColIf(SEP::Exp_C2, timeStampHistC1, timeStampHistT1);
  Animals_API::updateUpsampAnimalsColIf(Animals::EXP_C1, timeStampHistC1finish, timeStampHistT1finish);
}


extern "C" void SEP_copyAnimalsUpsampleData(int inst)
{
  // if (Animals_API::animalsCurrLCNight())
  //   return;

  Animals_API::copyAnimalsUpsampleData(inst);
}
