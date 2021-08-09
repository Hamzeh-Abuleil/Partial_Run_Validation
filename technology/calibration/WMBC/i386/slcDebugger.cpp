#include <cinttypes>
#ifdef MEwin

#define _IDBG_MACRO_DEFINITIONS

#include "slcDebugger.h"
#include "technology/calibration/WMBC/common/slc.h"
// #include "utilities/saddlePoints/saddlePoints_API.h"

#include "technology/mobilib/fix/common/MEXimage/win.h"
#include "technology/mobilib/fix/common/MEXimage/image.h"
#include "technology/mobilib/fix/common/MEXimage/coordinate.h"
#include "technology/mobilib/fix/common/MEXimage/colorPixel.h"
#include "basicTypes/show/common/drawingColorPixel.h"
// #include "technology/mobilib/fix/common/MEXimage/sync.h"
// #include "technology/mobilib/fix/common/MEXimage/typeImages.h"
#include "utilities/IDebugger/common/IDebugger_defs.h"
#include "utilities/IDebugger/common/IDebugger.h"
#include "utilities/IDebugger/common/IDebuggerDataGeneric.h"
#include "technology/calibration/utilities/cameraModel/cameraModel_API.h"
#include "technology/worldModel/common/worldModelUtils.h"


namespace WMBC {
  FOEFinder_Stationless_Debugger::FOEFinder_Stationless_Debugger(FOEFinder_Stationless* foeFinder_Stationless) :
    _foeFinder(foeFinder_Stationless)
  {  
  }

  IDBG::IDbgWindow* FOEFinder_Stationless_Debugger::createMainWin(std::string winName, Fix::MEimage::Size & sizeL1, const Fix::MEimage::Coordinate &pos, bool writePPMs)
  {
    IDBG::IDbgWindow* wnd = new IDBG::IDbgWindow(winName, 0, pos, sizeL1 * 4, writePPMs, false, true);
    wnd->addMenuItem(e_L_IMAGE, "Image", true);
    wnd->addMenuItem(e_L_SEARCH_AREA, "Search Area", false);
    wnd->addMenuItem(e_L_SP, "SP", false);
    wnd->addMenuItem(e_L_SEARCH_WIN, "Search Win", true);
    wnd->addMenuItem(e_L_TARGETS, "Targets", true);
    return wnd;
  }

  void FOEFinder_Stationless_Debugger::updateStage(Stage stage) {
    switch(stage) {
    case e_INIT:
      init();
      break;
    case e_START:
      start();
      break;
    case e_SEARCH_WIN:
      drawSearchWin();
      break;
    case e_TARGETS:
      drawTargets();
      break;
    case e_END:
      end();
      break;
    // case e_TEST:
    //   test();
    //   break;
    default:
      assert(0 && "FOEFinder_Stationless_Debugger:: Invalid Stage");
    }
  }

  // In STF the x,y fields in spInfo are remained distorted, while xSub,ySub are refined and undistorted

  void FOEFinder_Stationless_Debugger::init() {
    int verbosity = 0;
    bool colors = false;
    IDBG::IDebugger::instance(IDBG::WMBC).init("WMBC", "etc/wmbcDebugger.conf", &createMainWin, verbosity, &globalFrameIndex, colors);
  }

  void FOEFinder_Stationless_Debugger::start() {
    IDBG::IDebugger::instance(IDBG::WMBC).activateFrame();

    _img = _foeFinder->_data.algo.img;
    _left = _img->getObj().left();
    _bottom = _img->getObj().bottom();

    // Fix::MEimage::Image imgRect;
    // const WorldModel::CameraModel::IntrinsicCameraModel<float> *model = &WorldModel::CameraModel::intrinsicCameraModel<float>(-2);
    // WorldModel::rectifyI386(_img->getObj(), model, imgRect);
    // _leftRect = imgRect.left();
    // _bottomRect = imgRect.bottom();
    
    UPDATE_DEBUGGER(IDBG::WMBC, "slc", _img->getObj(), Image, e_L_IMAGE, Fix::MEimage::clrGreen);
  }

  void FOEFinder_Stationless_Debugger::drawSearchWin() {
    for (int i = 0; i < 2; ++i) {
      const Float::MEgeo::Rect sWin = _foeFinder->_targetSearchWindow[i]*2;
      UPDATE_DEBUGGER(IDBG::WMBC, "slc", sWin, fRect, e_L_SEARCH_WIN, Fix::MEimage::clrMagenta);
    }

    for (int i = 0; i < 3; i+=2) {
      float xpu = _foeFinder->_targetImgPosEstimator[i].getMeanPred();
      float ypu = _foeFinder->_targetImgPosEstimator[i+1].getMeanPred();
      float xcu = _foeFinder->_targetImgPosEstimator[i].getMean();
      float ycu = _foeFinder->_targetImgPosEstimator[i+1].getMean();
      float xpd=0, ypd=0, xcd=0, ycd=0;
      // _foeFinder->_distortion.distort(xpu, ypu, xpd, ypd);
      // _foeFinder->_distortion.distort(xcu, ycu, xcd, ycd);
      if (DistortionCorrectionAPI::isDistortionValid()) {
        DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, xpu, ypu, xpd, ypd);
        DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, xcu, ycu, xcd, ycd);
      }
      const Float::MEmath::Vec<2,float> pp(xpd, ypd);
      const Float::MEmath::Vec<2,float> pc(xcd, ycd);
      UPDATE_DEBUGGER2(IDBG::WMBC, "slc", pc, pp, Line, e_L_SEARCH_WIN, Fix::MEimage::clrCyan);
    }
  }

  void FOEFinder_Stationless_Debugger::drawTargets() {
    TargetDrawInfo *target = &(_foeFinder->_targetDraw);
    if (!target->valid) {
      return;
    }

    for (int i = 0; i < 2; ++i ) {
      const Float::MEgeo::Rect r = target->imCorners[i];
      UPDATE_DEBUGGER(IDBG::WMBC, "slc", r, fRect, e_L_TARGETS, Fix::MEimage::clrGreen);
    }
  }

  void FOEFinder_Stationless_Debugger::end() {
    IDBG::IDebugger::instance(IDBG::WMBC).frameDebug();
  }

  void FOEFinder_Stationless_Debugger::test() {
  }

} // namespace WMBC

#endif // MEwin
