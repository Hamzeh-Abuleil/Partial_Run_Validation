#include "technology/nightSpots/nightSpotsTracker_API.h"
#include "technology/nightSpots/nightConfig.h"
#include "technology/nightSpots/tracker_API.h"
#include "technology/spotsVD/spotsVDCoupling_API.h"
#include "technology/spotsVD/common/spotsVDCoupling_Internal_API.h"
#include "technology/spotsVD/common/spotFeaturesWrapper.h"
#include "technology/spotsVD/common/windowFeaturesWrapper.h"
#include "technology/brain2/brainDefaults.h"

#include "technology/objectSensing/objectDetection/VD2D/common/vclIgnoreAttention.h"
#include "technology/objectSensing/utilities/cameras/vcls_cameras.h"

#define SPOTS_VD_RUNNING_MODE_ACTIVE                  \
  {                                                   \
    if (Tracker_API::skipFrame() || !NightExposuresInfo::validExposures() || !VD2D_IS_RUNNING) \
      return;                                         \
                                                      \
    DUSK_IGNORE_SPOTS_ATTENTION;		      \
    NIGHT_IGNORE_SPOTS_ATTENTION;		      \
  }

// This file holds the SEPs of both spots and subSpots modules.

// spots features goals

extern "C" void SEP_vclUpdateSpotsFeatures(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotFeaturesWrapper::instance().updateNewFrame(SpotsVDConfiguration::synch().primarySpots(),
						 SpotsVDConfiguration::synch().mediumShortSpots(),
						 SpotsVDConfiguration::synch().veryShortSpots());
}

// spots windowing goals

extern "C" void SEP_vclInitCoupling(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotsVDCoupling_API::initCouplingEveryFrame();
}

extern "C" void SEP_vclCoupleShortestMFSpots(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotsVDCoupling_API::coupleShortestSpots();
}

extern "C" void SEP_vclFilterMediumShortMFSpots(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotsVDCoupling_API::filterMediumShortSpots();
}

extern "C" void SEP_vclCoupleMediumShortMFSpots(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotsVDCoupling_API::coupleMediumShortSpots();
}

extern "C" void SEP_vclFilterSLIMFSpots(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotsVDCoupling_API::filterSLISpots();
}

extern "C" void SEP_vclCoupleSLIMFSpots(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotsVDCoupling_API::coupleSLISpots();
}

extern "C" void SEP_vclCreateWindows(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotsVDCoupling_API::createWindows();
}

/*
extern "C" void SEP_vclCalcSpotsWindows(int instIdx)
{
  SpotsVDCoupling_API::calcSpotsWindows();
}
*/

extern "C" void SEP_vclUpdateWindowFeatures(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  WindowFeaturesWrapper::instance().updateNewFrame(SpotsVDCoupling_API::getSpotsWindowsInternal());
}

extern "C" void SEP_clearDaySpotsCouplingSFData(int instIdx)
{
  SPOTS_VD_RUNNING_MODE_ACTIVE
  SpotsVDCoupling_API::clearDaySpotsSFDataStructures();
  SpotsVDCoupling_API::clearVDOncoming();
}
