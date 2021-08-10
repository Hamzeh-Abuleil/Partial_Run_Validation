#include <cinttypes>
#ifdef MEwin
#include "technology/mobilib/std/me_math.h"

#define _IDBG_MACRO_DEFINITIONS

#include "stationlessTargetFinderDebugger.h"
#include "technology/calibration/WMBC/common/stationlessTargetFinder.h"
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
  StationlessTargetFinder_Debugger::StationlessTargetFinder_Debugger(StationlessTargetFinder* stationlessTargetFinder) :
    _targetFinder(stationlessTargetFinder)
  {  
    
  }

  IDBG::IDbgWindow* StationlessTargetFinder_Debugger::createMainWin(std::string winName, Fix::MEimage::Size & sizeL1, const Fix::MEimage::Coordinate &pos, bool writePPMs)
  {
    // IDBG::IDbgWindow* wnd = new IDBG::IDbgWindow("main", 0, pos, sizeL1 * 4, writePPMs, false, true);
    IDBG::IDbgWindow* wnd = new IDBG::IDbgWindow(winName, 0, pos, sizeL1 * 4, writePPMs, false, true);
    wnd->addMenuItem(e_L_IMAGE, "Image", true);
    wnd->addMenuItem(e_L_RAW_SP, "Raw SP", false);
    wnd->addMenuItem(e_L_POLES_CANDS, "Cands SP", false);
    wnd->addMenuItem(e_L_POLES_CHOSEN, "Chosen SP", false);
    wnd->addMenuItem(e_L_POLES_CLEAR, "Clear SP", false);
    wnd->addMenuItem(e_L_FINAL_TARGETS, "Final Targets", true);
    return wnd;
  }

  void StationlessTargetFinder_Debugger::updateStage(Stage stage) {
    switch(stage) {
    case e_INIT:
      init();
      break;
    case e_START:
      start();
      break;
    case e_RAW_SP:
      drawRawSp();
      break;
    case e_POLES_CANDS:
      drawCandsSp();
      break;
    case e_POLES_CHOSEN:
      drawPolesChosenSp();
      break;
    case e_FINAL_TARGETS:
      drawSpSubDist(*_inSp, e_L_POLES_CLEAR, Fix::MEimage::clrGreen);
      drawTargetCorners();
      drawFinalTargets();
      break;
    case e_END:
      end();
      break;
    case e_TEST:
      test();
      break;
    default:
      assert(0 && "StationlessTargetFinder_Debugger:: Invalid Stage");
    }
  }

  // In STF the x,y fields in spInfo are remained distorted, while xSub,ySub are refined and undistorted

  void StationlessTargetFinder_Debugger::drawSp(SPInfoVec &sp, Layer layer, Fix::MEimage::ColorPixel color) {
    unsigned int n = sp.size();
    for (unsigned int i = 0; i < n; ++i) {
      const Fix::MEimage::Coordinate ptd(sp[i].x, sp[i].y);
      UPDATE_DEBUGGER(IDBG::WMBC, "main", ptd, Point, layer, color);

      float ux = sp[i].x, uy = sp[i].y;
      if (DistortionCorrectionAPI::isDistortionValid()) {
        DistortionCorrectionAPI::rectifySafe(CameraInfo::e_FORWARD, -2, sp[i].x, sp[i].y, ux, uy);
      }
      const Float::MEmath::Vec<2,float> ptu(ux, uy);
      UPDATE_DEBUGGER(IDBG::WMBC, "mainRectified", ptu, fPoint, layer, color);
    }
  }

  void StationlessTargetFinder_Debugger::drawSpSub(SPInfoVec &sp, Layer layer, Fix::MEimage::ColorPixel color) {
    unsigned int n = sp.size();
    for (unsigned int i = 0; i < n; ++i) {
      // const Float::MEmath::Vec<2,float> pt(sp[i].xSub, sp[i].ySub);
      // UPDATE_DEBUGGER(IDBG::WMBC, "main", pt, fPoint, layer, color);
      const Float::MEgeo::Point2D pt(sp[i].xSub, sp[i].ySub);
      UPDATE_DEBUGGER_V(IDBG::WMBC, "main", SelectablefPoint, layer, pt, color, color, i);

      Fix::MEimage::Coordinate loc(_left + 10, _bottom + 30);
      char text[256];
      sprintf(text, "(%f, %f)", sp[i].xSub, sp[i].ySub);
      UPDATE_DEBUGGER_V(IDBG::WMBC, "main", Selected<IDBG::DebuggerDataText>, layer, i, text, loc, color);
    }
  }

  void StationlessTargetFinder_Debugger::drawSpSubDist(SPInfoVec &sp, Layer layer, Fix::MEimage::ColorPixel color) {
    unsigned int n = sp.size();
    for (unsigned int i = 0; i < n; ++i) {
      const Float::MEmath::Vec<2,float> ptu(sp[i].xSub, sp[i].ySub);
      UPDATE_DEBUGGER(IDBG::WMBC, "mainRectified", ptu, fPoint, layer, color);
      
      float dx=0, dy=0;
      // _targetFinder->_distortion->distort(sp[i].xSub, sp[i].ySub, dx, dy);
      if (DistortionCorrectionAPI::isDistortionValid()) {
        DistortionCorrectionAPI::unrectifySafe(CameraInfo::e_FORWARD, -2, sp[i].xSub, sp[i].ySub, dx, dy);
      }
      // const Float::MEmath::Vec<2,float> ptd(dx, dy);
      // UPDATE_DEBUGGER(IDBG::WMBC, "main", ptd, fPoint, layer, color);
      const Float::MEgeo::Point2D pt(dx, dy);
      UPDATE_DEBUGGER_V(IDBG::WMBC, "main", SelectablefPoint, layer, pt, color, color, i);

      Fix::MEimage::Coordinate loc(sp[i].x+10, sp[i].ySub);
      char text[256];
      sprintf(text, "undist: (%.2f, %.2f)", sp[i].xSub, sp[i].ySub);
      // UPDATE_DEBUGGER_V(IDBG::WMBC, "main", Selected<IDBG::DebuggerDataText>, layer, i, text, loc, color);
      UPDATE_DEBUGGER_V(IDBG::WMBC, "main", Selected<IDBG::DebuggerDataTextUpperCorner>, layer, i, text, loc, color);
    }
  }

  void StationlessTargetFinder_Debugger::setInputSp(SPInfoVec &sp) {
    _inSp = &sp;
  }

  void StationlessTargetFinder_Debugger::init() {
    int verbosity = 0;
    bool colors = false;
    IDBG::IDebugger::instance(IDBG::WMBC).init("WMBC", "etc/wmbcDebugger.conf", &createMainWin, verbosity, &globalFrameIndex, colors);
  }

  void StationlessTargetFinder_Debugger::start() {
    IDBG::IDebugger::instance(IDBG::WMBC).activateFrame();

    _img = _targetFinder->_img;
    _left = _img->getObj().left();
    _bottom = _img->getObj().bottom();

    Fix::MEimage::Image imgRect;
    // const WorldModel::CameraModel::IntrinsicCameraModel<float> *model = &WorldModel::CameraModel::intrinsicCameraModel<float>(-2);
    // WorldModel::rectifyI386(_img->getObj(), model, imgRect);
    WorldModel::rectifyI386(_img->getObj(), -2, CameraInfo::e_FORWARD, imgRect);
    _leftRect = imgRect.left();
    _bottomRect = imgRect.bottom();
    
    UPDATE_DEBUGGER(IDBG::WMBC, "main", _img->getObj(), Image, e_L_IMAGE, Fix::MEimage::clrGreen);
    UPDATE_DEBUGGER(IDBG::WMBC, "mainRectified", imgRect, Image, e_L_IMAGE, Fix::MEimage::clrGreen);

    const int BUFF_SIZE = 256;
    char text[BUFF_SIZE];
    sprintf(text, "origin: (%d, %d)", imgRect.origin().x, imgRect.origin().y);
    Fix::MEimage::Coordinate loc(_leftRect + 10, _bottomRect + 10);
    UPDATE_DEBUGGER2(IDBG::WMBC, "mainRectified", text, loc, Text, e_L_IMAGE, Fix::MEimage::clrGreen);

  }

  void StationlessTargetFinder_Debugger::drawRawSp() {
    drawSp(*_inSp, e_L_RAW_SP, Fix::MEimage::clrRed);

    const int BUFF_SIZE = 256;
    char text[BUFF_SIZE];
    sprintf(text, "#SP: %d; #minSP: %d", (int)_inSp->size(), 2*_targetFinder->_params.spNumInTarget);
    Fix::MEimage::Coordinate loc(_left + 10, _bottom + 10);
    UPDATE_DEBUGGER2(IDBG::WMBC, "main", text, loc, Text, e_L_RAW_SP, Fix::MEimage::clrRed);
  }

  void StationlessTargetFinder_Debugger::drawCandsSp() {
    drawSpSubDist(_targetFinder->_candsSp, e_L_POLES_CANDS, Fix::MEimage::clrYellow);

    const int BUFF_SIZE = 256;
    char text[BUFF_SIZE];
    sprintf(text, "#SP: %d; #minSP: %d", (int)_targetFinder->_candsSp.size(), 2*_targetFinder->_params.spNumInTarget);
    Fix::MEimage::Coordinate loc(_left + 10, _bottom + 50);
    UPDATE_DEBUGGER2(IDBG::WMBC, "main", text, loc, Text, e_L_POLES_CANDS, Fix::MEimage::clrYellow);
  }
    
  void StationlessTargetFinder_Debugger::drawPolesChosenSp() {
    const int POLE_NUM = 2;
    for (int i = 0; i < POLE_NUM; ++i) {
      drawSpSubDist(_targetFinder->_poles[i], e_L_POLES_CHOSEN, Fix::MEimage::clrOrange);
    }

    int actualPtNum[2];
    for (int i = 0; i < 2; ++i) {
      actualPtNum[i] = 0;
      int ptNum = (int)(_targetFinder->_poles[i].size());
      for (int j = 0; j < ptNum; ++j) {
        if (me_abs(_targetFinder->_poles[i][j].score) > 0.1) {
          actualPtNum[i]++;
        }
      }
    }

    const int BUFF_SIZE = 256;
    char text[BUFF_SIZE];
    sprintf(text, "#SP: %d + %d; #minSP: %d", actualPtNum[0], actualPtNum[1], 2*_targetFinder->_params.spNumInTarget);
    Fix::MEimage::Coordinate loc(_left + 10, _bottom + 90);
    UPDATE_DEBUGGER2(IDBG::WMBC, "main", text, loc, Text, e_L_POLES_CHOSEN, Fix::MEimage::clrOrange);
  }

  void StationlessTargetFinder_Debugger::drawTargetCorners() {
    int n = _targetFinder->_params.spNumInTarget;
    
    if ((int)_inSp->size() < 2*n) {
      return;
    }

    const int BUFF_SIZE = 256;
    char text[BUFF_SIZE];

    SaddlePoints::Types::SPInfo sp = (*_inSp)[0];
    Fix::MEimage::Coordinate loc;
    Float::MEgeo::Point2D pt0, pt1;

    int ind[4] = {0, n-1, 2*n-1, n}; // bottom-left, top-left, top-right, bottom-right
    for (int i = 0; i < 4; ++i) {
      if (i > 0) {
        pt0.X() = sp.xSub;
        pt0.Y() = sp.ySub;
      }
      sp = (*_inSp)[ind[i]];
      int sx = (i < 2) ? -100 : 20;
      int sy = (i < 1 || i > 2) ? -20 : 20;
      loc.x = me_lround(sp.xSub) + sx;
      loc.y = me_lround(sp.ySub) + sy;
      sprintf(text, "(%.2f, %.2f)", sp.xSub, sp.ySub);
      UPDATE_DEBUGGER2(IDBG::WMBC, "mainRectified", text, loc, Text, e_L_POLES_CLEAR, Fix::MEimage::clrGreen);
      if (i > 0) {
        pt1.X() = sp.xSub;
        pt1.Y() = sp.ySub;
        UPDATE_DEBUGGER2(IDBG::WMBC, "mainRectified", pt0, pt1, Line, e_L_POLES_CLEAR, Fix::MEimage::clrBlack);
      }
    }
    pt0.X() = (*_inSp)[0].xSub;
    pt0.Y() = (*_inSp)[0].ySub;
    UPDATE_DEBUGGER2(IDBG::WMBC, "mainRectified", pt0, pt1, Line, e_L_POLES_CLEAR, Fix::MEimage::clrBlack);

    loc.x = 0.5*((*_inSp)[0].xSub + (*_inSp)[n].xSub);
    loc.y = 0.5*((*_inSp)[0].ySub + (*_inSp)[n].ySub);
    float dx = (*_inSp)[0].xSub - (*_inSp)[n].xSub;
    float dy = (*_inSp)[0].ySub - (*_inSp)[n].ySub;
    float drb = sqrtf(dx*dx + dy*dy);
    sprintf(text, "dr=%.2f, dy=%.2f", drb, dy);
    UPDATE_DEBUGGER2(IDBG::WMBC, "mainRectified", text, loc, Text, e_L_POLES_CLEAR, Fix::MEimage::clrGreen);

    loc.x = 0.5*((*_inSp)[n-1].xSub + (*_inSp)[2*n-1].xSub);
    loc.y = 0.5*((*_inSp)[n-1].ySub + (*_inSp)[2*n-1].ySub);
    dx = (*_inSp)[n-1].xSub - (*_inSp)[2*n-1].xSub;
    dy = (*_inSp)[n-1].ySub - (*_inSp)[2*n-1].ySub;
    float drt = sqrtf(dx*dx + dy*dy);
    sprintf(text, "dr=%.2f, dy=%.2f", drt, dy);
    UPDATE_DEBUGGER2(IDBG::WMBC, "mainRectified", text, loc, Text, e_L_POLES_CLEAR, Fix::MEimage::clrGreen);

    loc.y = 0.5*((*_inSp)[0].ySub + (*_inSp)[n-1].ySub);
    sprintf(text, "ddr=%.2f", drb-drt);
    UPDATE_DEBUGGER2(IDBG::WMBC, "mainRectified", text, loc, Text, e_L_POLES_CLEAR, Fix::MEimage::clrGreen);
  }

  void StationlessTargetFinder_Debugger::drawFinalTargets() {
    int n = _targetFinder->_params.spNumInTarget;
    
    if ((int)_inSp->size() < 2*n) {
      return;
    }

    for (int i = 0; i < 2; ++i ) {
      int ib = i*n;
      int it = (i+1)*n - 1;
      float s = ((*_inSp)[it].y - (*_inSp)[ib].y)*1.f/(n-1);
      float left = (*_inSp)[ib].x - s;
      float right = (*_inSp)[ib].x + s;
      float bottom = (*_inSp)[ib].y - s;
      float top = (*_inSp)[it].y + s;

      const Float::MEgeo::Rect target(left, right, bottom, top);
      UPDATE_DEBUGGER(IDBG::WMBC, "main", target, fRect, e_L_FINAL_TARGETS, Fix::MEimage::clrGreen);
    }
  }

  void StationlessTargetFinder_Debugger::end() {
    IDBG::IDebugger::instance(IDBG::WMBC).frameDebug();
  }

  void StationlessTargetFinder_Debugger::test() {
    std::cout << "test" << std::endl;
  }

  

} // namespace WMBC

#endif // MEwin
