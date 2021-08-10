/****************************************************************
 * WMBC_API.cpp
 *
 ****************************************************************/

#include "technology/calibration/WMBC/common/wmbc_API_internal.h"
#include "technology/calibration/WMBC/common/wmbc.h"
#include "technology/calibration/WMBC/common/slc.h"
#include "technology/calibration/WMBC/common/wmbcProperties.h"
#include "technology/calibration/WMBC/common/wmbcTypes.h"
#include "functionality/calibration/propertiesManager.h"
#include "technology/calibration/WMBC/common/wmbc_dbg.h"
#include "functionality/partialRun/partialRun_API.h"

extern int g_mevsa_brain_argc;
extern char** g_mevsa_brain_argv;

extern bool LoadProperDies(Properties& p, const char* path, const char* sec = NULL);

namespace WMBC {

  static FOEFinder *g_foeFinder = nullptr;
  static FOEFinder *g_foeFinderFS = nullptr;
  static WmbcProperties* g_wmbcProperties = nullptr;

  void initWMBC() {
    CHECK_FORWARD_CAM
    ASSERT(!g_foeFinder && !g_wmbcProperties);
    if (g_foeFinder || g_wmbcProperties) {
      closeWMBC();
    }

    bool loadOk = initProperties();
    if (!loadOk) {
      return;
    }

    ASSERT(!g_foeFinder && g_wmbcProperties);

    if (!g_wmbcProperties->wmbcRun()) {
      return;
    }

    if (g_foeFinder) {
      return;
    }

    switch (g_wmbcProperties->wmbcMode()) {
    case WmbcProperties::e_Run_Autofix:
      g_foeFinder  = new FOEFinder_Autofix(g_wmbcProperties);
      break;
    case WmbcProperties::e_Run_SPC:
      g_foeFinder  = new FOEFinder_SPC(g_wmbcProperties);
      break;
    case WmbcProperties::e_Run_SLC:
      g_foeFinder  = new FOEFinder_Stationless(g_wmbcProperties);
      break;
    default:
      g_foeFinder  = new FOEFinder(g_wmbcProperties);
      break;
    }
    auto wmbcIF_wrapper = g_foeFinder->getWmbcIF_wrapper();
    if (wmbcIF_wrapper != nullptr){
      PartialRun_API::registerIF(wmbcIF_wrapper);
    }
    auto autoFixIF_wrapper = g_foeFinder->getAutoFixIF_wrapper();
    if (autoFixIF_wrapper != nullptr){
      PartialRun_API::registerIF(autoFixIF_wrapper);
    }
  }

  void initImages(const Prep::SafeImg* imgDist, const Prep::SafeImg* imgTargets, int level) {
    if (!isRunning()) {
      return;
    }
    g_foeFinder->initImages(imgDist, imgTargets, level);
    if (isRunningAutofix()) {
      // ASSERT(g_foeFinderFS || (g_wmbcProperties && !g_wmbcProperties->wmbcRun()));
      // g_foeFinderFS->initImages(imgDist, imgTargets, level);
    }
  }

  bool initProperties() {
    ASSERT(!g_wmbcProperties);
    if (!g_wmbcProperties) {
      g_wmbcProperties = new WmbcProperties();
    }

    bool ok = LoadProperDies(*g_wmbcProperties, "etc/wmbc.conf", "[WMBC_CONF]");
    // if (!ok) {
    //   return ok;
    // }
    g_wmbcProperties->load(g_mevsa_brain_argc, g_mevsa_brain_argv);
    ok = g_wmbcProperties->loaded();
    //if (!ok) {
    //  return ok;
   // }
    PropertiesManager& propertiesManager = getCalibrationManager();
    propertiesManager.push_back(g_wmbcProperties, "wmbcProperties");
    return ok; // && g_wmbcProperties->loaded();
  }

  void runSlcTargets() { // TODO: change void to error/status
    if (!isRunningSLC()) {
      return;
    }

    WMBC_DTIMER(apiTimer);
    WMBC_STIMER(apiTimer);
    g_foeFinder->runTargets();
    WMBC_ETIMER(apiTimer, "[runSlcTargets@wmbc_API]");
  }

  void runWMBC() { // TODO: change void to error/status
    if (!isRunning()) {
      return;
    }
    WMBC_DTIMER(wmbcTimer);
    WMBC_STIMER(wmbcTimer);
    g_foeFinder->run();
    if (isRunningAutofix()) {
      // g_foeFinderFS->run();
    }
    WMBC_ETIMER(wmbcTimer, "[runWMBC@wmbc_API]");
  }

  void prepareInputEndFrame() {
    if (!isRunning()) {
      return;
    }
    g_foeFinder->setInputDataEndFrame();
  }

  void fillModelIF() {
    if (!isRunning()) {
      return;
    }
    g_foeFinder->fillModelIF();
    if (isRunningAutofix()) {
      // g_foeFinderFS->fillModelIF();
    }
  }

  PartialRun::MultiCameraAutoFixIF_wrapper* getAutoFixIFWrapper() {
    if (!isRunning()) {
      return nullptr;
    }
    return g_foeFinder->getAutoFixIF_wrapper();
  }

  PartialRun::WmbcIF_wrapper* getWmbcIF_wrapper() {
    if (!isRunning()) {
      return nullptr;
    }
    return g_foeFinder->getWmbcIF_wrapper();
  }

  Fix::MEimage::Sync<WmbcIF>* getWmbcIF() {
    if (!isRunning()) {
      return nullptr;
    }
    return g_foeFinder->getWmbcIF();
  }

  EWmbcIF* getEWmbcIF() {
    if (!isRunning()) {
      return nullptr;
    }
    return g_foeFinder->getEWmbcIF();
  }

  Fix::MEimage::Sync<WmbcIF>* getQuickWmbcIF() {
    if (!isRunningAutofix()) {
      return nullptr;
    }
    return g_foeFinderFS->getWmbcIF();
  }

  Fix::MEimage::Sync<MultiCameraAutoFixIF>* getAutoFixIF() {
    if (!isRunningAutofix()) {
      return nullptr;
    }
    return g_foeFinder->getAutoFixIF();
  }

  EMultiCameraAutoFixIF* getEAutoFixIF() {
    if (!isRunningAutofix()) {
      return nullptr;
    }
    return g_foeFinder->getEAutoFixIF();
  }

  void fillAutofixFailSafe(bool &autoFixFailSafe, bool &autoFixFailSafeYaw, bool &autoFixFailSafeHorizon) {
    g_foeFinder->fillAutofixFailSafe(autoFixFailSafe, autoFixFailSafeYaw, autoFixFailSafeHorizon);
  }

  bool isRunning() {
    if (!g_foeFinder || !g_wmbcProperties) {
      return false;
    }

    return g_wmbcProperties->wmbcRun();
  }

  bool isRunningAutofix() {
    return isRunning() && (g_wmbcProperties->wmbcMode() == WmbcProperties::e_Run_Autofix);
  }

  bool isRunningSPC() {
    return isRunning() && (g_wmbcProperties->wmbcMode() == WmbcProperties::e_Run_SPC);
  }

  bool isRunningSLC() {
    return isRunning() && (g_wmbcProperties->wmbcMode() == WmbcProperties::e_Run_SLC);
  }

  void closeWMBC() {
    ASSERT(g_foeFinder && g_wmbcProperties);
    if (g_foeFinder) {
      delete g_foeFinder;
      g_foeFinder = nullptr;
    }
    if (g_foeFinderFS) {
      delete g_foeFinderFS;
      g_foeFinderFS = nullptr;
    }
    if (g_wmbcProperties) {
      delete g_wmbcProperties;
      g_wmbcProperties = nullptr;
    }
  }

  // confidence for REM
  // wmbc getters

  float confidence() {
    if (!isRunning()) {
      return 0.f;
    }
    return g_foeFinder->getConfidence();
  }

  int confidenceGrade() {
    if (!isRunning()) {
      return 0;
    }
    return g_foeFinder->getConfidenceGrade();
  }

  void getFoeInExpAllocated(CameraInfo::CameraInstance cinst, PrepSys::exp_mask exp, float& x, float& y, bool interim/*=false*/) {
    if (!CameraInfo::calibrationExists(cinst)) {
      return;
    }
    if (!isRunning()) {
      PrepSys_API::getFoeInExpAllocated(cinst, exp, x, y);
      return;
    }

    const PixelLm2_f foeLm2 = g_foeFinder->foeAllocLm2(cinst, interim);
    x = foeLm2.X();
    y = foeLm2.Y();
  }

  // autofix getters
  int  autoFix_yawDelta(CameraInfo::CameraInstance inst) {
    if (!isRunningAutofix()) {
      return 0;
    }
    if (!CameraInfo::exists(inst) || inst != CameraInfo::e_FORWARD) {
      return 0;
    }

    return g_foeFinder->getYawDelta();
  }

  int  autoFix_horizonDelta(CameraInfo::CameraInstance inst) {
    if (!isRunningAutofix()) {
      return 0;
    }
    if (!CameraInfo::exists(inst) || inst != CameraInfo::e_FORWARD) {
      return 0;
    }

    return g_foeFinder->getHorizonDelta();
  }

  bool yawConverged(CameraInfo::CameraInstance inst) {
    if (!isRunningAutofix()) {
      return false;
    }

    return g_foeFinder->getYawConvergence();
  }
  
  bool horizonConverged(CameraInfo::CameraInstance inst) {
    if (!isRunningAutofix()) {
      return false;
    }

    return g_foeFinder->getHorizonConvergence();
  }
  
  

} // namespace WMBC
  




