#include "technology/calibration/CirclesTAC/API.h"
#include "technology/brain2/brain2_API.h"
#include "technology/calibration/CirclesTAC/circlestacImageInfo.h"
#include "technology/calibration/CirclesTAC/CirclesTAC_ServicesUser.h"

extern "C"
{

  // not working
  void INIT_CirclesTAC()
  {
    MEImageInfo::ImageInfoCirclesTAC::instance().init();
    CirclesTACServicesUser::instance().init();
    CirclesTAC::init();
    Brain2API::setCirclesTACIF(CirclesTAC::getModelIF(), CirclesTAC::getEModelIF());
  }
  
  void SEP_CirclesTACRunMe()
  {
    const Prep::SafeImg* textureIm = MEImageInfo::ImageInfoCirclesTAC::instance().getImageInfo({ MEImageInfo::IMAGE_REGULAR, CirclesTAC::SLOW, 0})->getSafeImage(-1);
    ASSERT(textureIm && textureIm->available());


    int fs_was_enabled = gsf_enable_fs(1);
    CirclesTAC::run(**textureIm);
    gsf_enable_fs(fs_was_enabled);
  }  
  
}
