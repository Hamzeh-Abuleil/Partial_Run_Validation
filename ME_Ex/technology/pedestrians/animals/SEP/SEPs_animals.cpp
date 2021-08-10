#include "technology/pedestrians/animals/Animals_API.h"
#include "technology/pedestrians/animals/animalsSingleFrame/animalsSingleFrame_API.h"
#include "technology/pedestrians/animals/animalsSingleFrame/common/animalsSingleFrameManager_API_internal.h"
#include "technology/pedestrians/animals/animalsCliquer/common/cliquer_API_internal.h"
#include "technology/pedestrians/animals/animalsTracking/animalsTracking_API.h"
#include "technology/pedestrians/animals/common/animalsCommonResources.h"

#include "technology/matching/radarMatching_API.h"

#include "technology/include/SEP/SEP_InstIdx.h"



//extern int globalFrameIndex;
extern int globalBrainIndex;

//---------------- Start Frame -------------------------------

extern "C" void SEP_animalsStartFrame(int instIdx)
{
  Animals_API::animalsStartFrame();
}

//---------------- FFT Goals -------------------------------

extern "C" void SEP_prepAnimalsFFTGradient(int instIdx)
{

  if (!AnimalsCommonResources::instance().runAnimals() || AnimalsCommonResources::instance().enableLoadBalance())
  {
    return;
  }
  
  switch (instIdx) 
  {
    case SEP::Lm2:
    {
      Animals_API::prepareFFTGradient(-2);
      break;
    }
    case SEP::Lm1:
    {
      Animals_API::prepareFFTGradient(-1);
      break;
    }
    case SEP::L0:
    {
      Animals_API::prepareFFTGradient(0);
      break;
    }
    case SEP::L1:
    {
      Animals_API::prepareFFTGradient(1);
      break;
    }
    case SEP::L2:
    {
      Animals_API::prepareFFTGradient(2);
      break;
    }
    default:
    {
      assert(false);
      break;
    }
  }
}

extern "C" void SEP_prepAnimalsFFT(int instIdx)
{
  if (!AnimalsCommonResources::instance().runAnimals() || AnimalsCommonResources::instance().enableLoadBalance())
  {
    return;
  }

  switch (instIdx) 
  {
    case SEP::Lm2:
    {
      Animals_API::prepareFFT(-2);
      break;
    }
    case SEP::Lm1:
    {
      Animals_API::prepareFFT(-1);
      break;
    }
    case SEP::L0:
    {
      Animals_API::prepareFFT(0);
      break;
    }
    case SEP::L1:
    {
      Animals_API::prepareFFT(1);
      break;
    }
    case SEP::L2:
    {
      Animals_API::prepareFFT(2);
      break;
    }
    default:
    {
      assert(false);
      break;
    }
  }
}

extern "C" void SEP_postAnimalsFFT(int instIdx)
{  
  if (!AnimalsCommonResources::instance().runAnimals() || AnimalsCommonResources::instance().enableLoadBalance())
  {
    return;
  }
  
  switch (instIdx) 
  {
    case SEP::Lm2:
    {
      Animals_API::postFFT(-2);
      break;
    }
    case SEP::Lm1:
    {
      Animals_API::postFFT(-1);
      break;
    }
    case SEP::L0:
    {
      Animals_API::postFFT(0);
      break;
    }
    case SEP::L1:
    {
      Animals_API::postFFT(1);
      break;
    }
    case SEP::L2:
    {
      Animals_API::postFFT(2);
      break;
    }
    default:
    {
      assert(false);
      break;
    }
  }
}


//---------------- Animals Single Frame --------------------

extern "C" void SEP_animalsSingleFrameCreateDetected()
{
//std::cerr << "in " << __FUNCTION__ << std::endl;
  Animals::createDetected();
}

//-----------------------  DAY  ----------------------------

// Phase 0 (Intensity Classifier)
extern "C" void SEP_prepAnimalsSFPhase0()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::prepAnimalsSFDayPhase0();
  }
  else {
    Animals::prepAnimalsSFNightPhase0();
  }

}

extern "C" void SEP_postAnimalsSFPhase0()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::postAnimalsSFDayPhase0();
  }
  else  {
    Animals::postAnimalsSFNightPhase0();
  }
}


// Phase 1 (VHFB Classifier)
extern "C" void SEP_prepAnimalsSFPhase1()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::prepAnimalsSFDayPhase1();
  }
  else {
    Animals::prepAnimalsSFNightPhase1();
  }
}

extern "C" void SEP_postAnimalsSFPhase1()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::postAnimalsSFDayPhase1();
  }
  else {
    Animals::postAnimalsSFNightPhase1();
  }
}

// Align (VHFB classifiers)
extern "C" void SEP_prepAnimalsSFAlignPhase0()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::prepAnimalsSFDayAlignPhase0();
  }
  else {
    Animals::prepAnimalsSFNightAlignPhase0();
  }

}

extern "C" void SEP_postAnimalsSFAlignPhase0()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::postAnimalsSFDayAlignPhase0();
  }
  else {
    Animals::postAnimalsSFNightAlignPhase0();
  }
}

// Align Top (VHFB classifier)
extern "C" void SEP_prepAnimalsSFAlignPhase1()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::prepAnimalsSFDayAlignPhase1();
  }
  else {
    Animals::prepAnimalsSFNightAlignPhase1();
  }
}

extern "C" void SEP_postAnimalsSFAlignPhase1()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::postAnimalsSFDayAlignPhase1();
  }
  else {
    Animals::postAnimalsSFNightAlignPhase1();
  }
}

// Phase 1 (VHFB Classifier)
extern "C" void SEP_prepAnimalsSFPhase2()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::prepAnimalsSFDayPhase2();
  }
  else {
    Animals::prepAnimalsSFNightPhase2();
  }
}

extern "C" void SEP_postAnimalsSFPhase2()
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::postAnimalsSFDayPhase2();
  }
  else {
    Animals::postAnimalsSFNightPhase2();
  }
}


extern "C" void SEP_animalsCollectFinalSf()
{
  Animals::collectFinalSf();
}


//---------------- Animals Multi Frame ---------------------

// Predictions
extern "C" void SEP_animalsPredictKalman(int instIdx)
{

}

extern "C" void SEP_animalsUpdatePredictionData(int instIdx)
{

}

// Tracking
extern "C" void SEP_applyAnimalsClqTracking(int instIdx)
{
  Animals::prepAnimalsClqTracking();
  Animals::postAnimalsClqTracking();
}

// MF alignment for the tracked rect
extern "C" void SEP_prepAnimalsMFAlign()
{
  Animals::prepAnimalsMFDayAlign();
}

extern "C" void SEP_postAnimalsMFAlign()
{
  Animals::postAnimalsMFDayAlign();
}

// cliques

extern "C" void SEP_animalsCreateCliques(int instIdx)
{
//  if (Animals_API::animalsCurrLCDay() ) {
    Animals::animalsCreateCliques();
//  }
}

// cliques classifiers

extern "C" void SEP_prepAnimalsMfLbp(int instIdx)
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::prepAnimalsMfDayLbp();
  }
  else
  {
    Animals::prepAnimalsMfNightLbp();
  }
}

extern "C" void SEP_postAnimalsMfLbp(int instIdx)
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::postAnimalsMfDayLbp();
  }
  else
  {
    Animals::postAnimalsMfNightLbp();
  }
}

extern "C" void SEP_prepAnimalsMfVHFB(int instIdx)
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::prepAnimalsMfDayVHFB();
  }
  else
  {
    Animals::prepAnimalsMfNightVHFB();
  }
}

extern "C" void SEP_postAnimalsMfVHFB(int instIdx)
{
  if (Animals_API::animalsCurrLCDay() ) {
    Animals::postAnimalsMfDayVHFB();
  }
  else
  {
    Animals::postAnimalsMfNightVHFB();
  }
}

// other scores


extern "C" void SEP_animalsCliquesPostClassifiers(int instIdx)
{
//  if (Animals_API::animalsCurrLCDay() ) {
  Animals::animalsCliquesPostClassifiers();
//  }
}

extern "C" void SEP_animalsCliquesOtherScores(int instIdx)
{
  Animals::animalsCliquesOtherScores();
}

extern "C" void SEP_animalCliquesMeasurements(int instIdx)
{
  Animals::animalCliquesMeasurements();
}

//---------------- End Frame -------------------------------


extern "C" void SEP_updateAnimalsLCDecision()
{
  Animals_API::updateAnimalsLCDecision();
}

extern "C" void SEP_animalsManageApproved()
{
  Animals_API::manageApproved();
}

extern "C" void SEP_animalsGetVDObjects()
{
  Animals_API::getVDObjects();
}

extern "C" void SEP_animalsEndFrame(int instIdx)
{
  Animals_API::animalsEndFrame();
  Animals_API::animalsUpdateLightingConditionsForFFT();
}

extern "C" void SEP_animalsFillColIF(int instIdx)
{
  Animals_API::animalsFillColIF();
}

extern "C" void SEP_animalsCheckOverlap(int instIdx)
{
//  Animals_API::animalsCheckOverlap();
}



// ------------ radar matching ----------------------------
extern "C" void SEP_animalsMatching()
{

  bool fusionEnable = Radar::e_FUSION == RadarMatching_API::getMode();
  static const bool matchOnly = Debug::Args::instance().existsParameter("-sMatchOnly");

  if ((!fusionEnable && !matchOnly) || globalBrainIndex < 1) {
    return;
  }

  MEtypes::ptr_vector<Radar::AnimalObstacle>& animals = RadarMatching_API::getAnimalApproved();
  MEtypes::ptr_vector<Radar::AnimalObstacle>& animalsSuspects = RadarMatching_API::getAnimalSuspects();
  animals.clear();
  animalsSuspects.clear();

  Animals_API::getAnimalRadarMatches(animals, animalsSuspects);

  RadarMatching_API::calcAnimals(globalBrainIndex, animals, animalsSuspects);

  if (!matchOnly){
    Animals_API::setAnimalRadarMatches (animals, animalsSuspects);
  }

  Animals_API::addAnimalsMatchingToItrk();
}
