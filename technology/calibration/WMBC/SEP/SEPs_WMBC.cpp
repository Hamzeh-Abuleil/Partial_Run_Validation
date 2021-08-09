/*
 * SEPs_WMBC.cpp
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

//#include "technology/calibration/WMBC/wmbc_API.h"
#include "technology/calibration/WMBC/common/wmbc_API_internal.h"
#include "technology/brain2/brain2_API.h"
// #include "technology/include/SEP/SEP_InstIdx.h"
#include "functionality/partialRun/partialRun_API.h"

extern "C" void SEP_runWMBC(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::WMBC);
  WMBC::runWMBC();
}

extern "C" void SEP_wmbcPrepareInputEndFrame(int instIdx)
{
  auto wmbcIF_wrapper = WMBC::getWmbcIF_wrapper();
  auto autoFixIF_wrapper = WMBC::getAutoFixIFWrapper();

  PartialRun_API::updatePartialRunLoad(wmbcIF_wrapper, PartialRun::IFCategory::WMBC_PR, 0);
  PartialRun_API::updatePartialRunLoad(autoFixIF_wrapper, PartialRun::IFCategory::WMBC_PR, 0);

  if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::WMBC)){
  WMBC::prepareInputEndFrame();
  }

  PartialRun_API::updatePartialRunStore(wmbcIF_wrapper, PartialRun::IFCategory::WMBC_PR, 0);
  PartialRun_API::updatePartialRunStore(autoFixIF_wrapper, PartialRun::IFCategory::WMBC_PR, 0);
}

extern "C" void SEP_wmbcFillModelIF(int instIdx)
{
  auto wmbcIF_wrapper = WMBC::getWmbcIF_wrapper();
  auto autoFixIF_wrapper = WMBC::getAutoFixIFWrapper();

  PartialRun_API::updatePartialRunLoad(wmbcIF_wrapper, PartialRun::IFCategory::WMBC_PR, 1);
  PartialRun_API::updatePartialRunLoad(autoFixIF_wrapper, PartialRun::IFCategory::WMBC_PR, 1);

  if (!PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType::WMBC)){
    WMBC::fillModelIF();
  }

  PartialRun_API::updatePartialRunStore(wmbcIF_wrapper, PartialRun::IFCategory::WMBC_PR, 1);
  PartialRun_API::updatePartialRunStore(autoFixIF_wrapper, PartialRun::IFCategory::WMBC_PR, 1);
}

extern "C" void SEP_slcFindTargets(int instIdx)
{
  RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::WMBC);
  ApplicationMode::applicationMode_t appMode = Brain2API::applicationData().getMode();
  if (appMode == ApplicationMode::STATIONLESS_TAC) {
    WMBC::runSlcTargets();
  }
}
