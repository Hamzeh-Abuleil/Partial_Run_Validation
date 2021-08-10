/*
 * WMBC_API.h
 *
 *  Created on: Feb 27, 2019
 *      Author: urilo
 */

#ifndef WMBC_API_INTERNAL_H_
#define WMBC_API_INTERNAL_H_


//#include "technology/calibration/WMBC/interface/wmbcIF.h"
//#include "technology/mobilib/fix/common/MEXimage/sync.h"
// #include "technology/brain2/prepSys/prepSys_API.h"
#include "technology/mobilib/fix/common/MEXimage/typeImages.h"
#include "functionality/partialRun/wrappers/WmbcIF_wrapper.h"
#include "functionality/partialRun/wrappers/MultiCameraAutoFixIF_wrapper.h"
//struct MultiCameraAutoFixIF;
//struct EMultiCameraAutoFixIF;

namespace WMBC {

  //struct TargetInfo;
  struct TargetInfo;

  // IF
  Fix::MEimage::Sync<WmbcIF>* getWmbcIF();
  EWmbcIF* getEWmbcIF();
  Fix::MEimage::Sync<WmbcIF>* getQuickWmbcIF();
  
  // running mode status
  bool isRunning();
  bool isRunningAutofix();
  bool isRunningSPC();
  bool isRunningSLC();

  // confidence for REM
  float confidence();
  int confidenceGrade();

  // foe for WorldModel::CameraModel
  void getFoeInExpAllocated(CameraInfo::CameraInstance cinst, PrepSys::exp_mask exp, float& x, float& y, bool interim=false);

  // autofix getters
  int  autoFix_yawDelta(CameraInfo::CameraInstance inst);
  int  autoFix_horizonDelta(CameraInfo::CameraInstance inst);
  bool yawConverged(CameraInfo::CameraInstance inst);
  bool horizonConverged(CameraInfo::CameraInstance inst);
  Fix::MEimage::Sync<MultiCameraAutoFixIF>* getAutoFixIF();
  EMultiCameraAutoFixIF* getEAutoFixIF();
  
  void initWMBC();
  void initImages(const Prep::SafeImg* imgDist, const Prep::SafeImg* imgTargets, int level);
  bool initProperties();
  void runSlcTargets();
  void runWMBC();
  void prepareInputEndFrame();
  void fillModelIF();
  void closeWMBC();
  PartialRun::MultiCameraAutoFixIF_wrapper* getAutoFixIFWrapper();
  PartialRun::WmbcIF_wrapper* getWmbcIF_wrapper();
  void fillAutofixFailSafe(bool &autoFixFailSafe, bool &autoFixFailSafeYaw, bool &autoFixFailSafeHorizon);

}


#endif /* WMBC_API_INTERNAL_H */
