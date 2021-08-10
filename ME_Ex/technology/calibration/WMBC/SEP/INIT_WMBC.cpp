/*
 * INIT_WMBC.cpp
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

#include "technology/calibration/WMBC/common/wmbc_API_internal.h"
#include "technology/calibration/WMBC/common/wmbc_API_internal.h"
#include "technology/brain2/brain2_API.h"
#include "technology/calibration/WMBC/common/wmbcService.h"
#include "technology/calibration/WMBC/wmbc_ServicesUser.h"
// #include "functionality/interface/applicationData.h"


extern "C" void INIT_WMBC(int instIdx)
{
  WmbcService * wService=new WmbcService;
  ServiceLocator_API::ServiceLocator::instance()->advertise(wService);
  WmbcServicesUser::instance().init();
  WMBC::initWMBC();
  Brain2API::setWmbcIF(WMBC::getWmbcIF(), WMBC::getEWmbcIF());
}
