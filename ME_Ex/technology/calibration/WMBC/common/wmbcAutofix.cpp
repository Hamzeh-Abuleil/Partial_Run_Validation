/*
 * wmbcAutofix.cpp
 *
 *  Created on: Nov 16, 2019
 *      Author: urilo
 */

#include "wmbc.h"
#include "functionality/interface/modelIF.h"
#include "technology/calibration/cameraToCamera/cameraToCamera_API_internal.h"
#include "utilities/cameraInformation/cameraInformation_API.h"
#include "utilities/cameraInformation/common/cameraProperties.h"
#include "technology/calibration/WMBC/common/wmbcOutputIF.h"
#include "utilities/autoFix/autoFix_API.h"
#include "utilities/autoFix/common/autoFixProperties.h"
#include "technology/calibration/WMBC/common/wmbc_dbg.h"
#include "technology/calibration/WMBC/common/wmbcUtils.h"

#define WMBC_CAM_EXIST(i) (CameraInfo::exists((CameraInfo::CameraInstance)i))
#define WMBC_CAM_CEXIST(i) (CameraInfo::calibrationExists((CameraInfo::CameraInstance)i))
#define WMBC_CAM_OK(i) (WMBC_CAM_EXIST(i) && WMBC_CAM_CEXIST(i))
#define WMBC_CAM_SKIP(i) if (!WMBC_CAM_OK(i)) { continue;}

namespace WMBC {

  FOEFinder_Autofix::FOEFinder_Autofix(WmbcProperties *properties) : FOEFinder(properties), _autoFixIF_wrapper(&_autoFixIF)  {
  
    MultiCameraAutoFixIF *m = &_autoFixIF.editable();
    for(unsigned i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i){
      m->multiAutoFix[i].wasInitialized = true;
    }
    _autoFixIF.update();
    initAutofixState();
  }

  void FOEFinder_Autofix::initAutofixState() {
    for (int i=0; i<2; ++i) {
      _autofixState.foeOutOfLimit[i] = false;
      _autofixState.foeCurrIsOk[i]= false;
    }
    for(unsigned i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i) {
      WMBC_CAM_SKIP(i)
      initAutofixState(i);
    }
  }

  void FOEFinder_Autofix::initAutofixState(int idx) {
    CameraInfo::CameraInstance inst = (CameraInfo::CameraInstance)idx;
    if (!CameraInfo::exists(inst)) {
      return;
    }
    AutofixLimits &l = _autofixState.limits[idx];

    // value of autofix read from etc/can on ignition in level 0 (this is relative to init calibration)
    l.foeAutofixL0Init[e_YAW] = CameraInfo::initialAutoFix_yaw(inst); // todo: init -> prevCycle
    l.foeAutofixL0Init[e_HORIZON] = CameraInfo::initialAutoFix_horizon(inst);

    // init calibration (TAC/SPC...) in level 0 (this is relative to nominal center of image)
    l.foeL0Init[e_YAW] = CameraInfo::yaw(inst) - l.foeAutofixL0Init[e_YAW];
    l.foeL0Init[e_HORIZON] = CameraInfo::horizon(inst) - l.foeAutofixL0Init[e_HORIZON];

    l.currIsOkThreshold = 2.5f; // [px level 0] equal to MIN_AUTOFIX_LARGE_DELTA in AutoFix

    for (int i=0; i<2; ++i) {
      // limits in level 0. If exceeded "out of calibration failsafe" is raised
      l.foeLowerLimitL0[i] = i==e_YAW ? CameraInfo::minYaw(inst) : CameraInfo::minHorizon(inst);
      l.foeUpperLimitL0[i] = i==e_YAW ? CameraInfo::maxYaw(inst) : CameraInfo::maxHorizon(inst);

      l.foeOutOfLimit[i] = false;

      // is the current value converging close enough to current calibration in FFS
      l.foeCurrIsOk[i] = false;

      // current calibration in FFS
      l.foeAutofixDeltaL0LastUpdated[i] = 0;
    }

    // override range params for wmbc functionality
    if (idx != (int)CameraInfo::e_FORWARD) {
      return;
    }
    MetaParams &mp            = _data.metaParams;
    if (!mp.useSpecialLimits) {
      mp.foeRange[e_YAW][0]     = 4*l.foeLowerLimitL0[e_YAW];
      mp.foeRange[e_YAW][1]     = 4*l.foeUpperLimitL0[e_YAW];
      mp.foeRange[e_HORIZON][0] = 4*l.foeLowerLimitL0[e_HORIZON];
      mp.foeRange[e_HORIZON][1] = 4*l.foeUpperLimitL0[e_HORIZON];
    }
  }

  void FOEFinder_Autofix::setResults() {
    FOEFinder::setResults();

    for (int i=0 ; i < CameraInfo::e_NUM_OF_CAM_INSTANCES ; ++i) {
      WMBC_CAM_SKIP(i)
      setAutofixOutput(i);
      // option to raise FS if any of the cam exceeds limits
      // AutofixLimits &l = _autofixState.limits[i];
      // for (int j=0; j<2; ++j) {
      //   _autofixState.foeOutOfLimit[j] = _autofixState.foeOutOfLimit[j] || l.foeOutOfLimit[j];
      //   _autofixState.foeCurrIsOk[j]   = _autofixState.foeCurrIsOk[j] || l.yawCurrIsOk[j];
      // }
    }

    // currently like old autofix decide FS only by e_FORWARD
    int insti = (int)(CameraInfo::e_FORWARD);
    AutofixLimits &l                = _autofixState.limits[insti];
    for (int j=0; j<2; ++j) {
      _autofixState.foeOutOfLimit[j] = l.foeOutOfLimit[j];
      _autofixState.foeCurrIsOk [j]  = l.foeCurrIsOk[j];
    }
  }

  void FOEFinder_Autofix::setAutofixOutput(int idx) {
    // When yaw/horizon converges Autofix container values are updated then:
    // Tested if Failsafe values are exceeded.
    // If they do they are chopped to FS boundaries.
    // If the correction does not exceed FS, but is still large (by angle measure)
    // it is chopped to not rock the boat to technologies which are used to old Autofix
    AutofixOutput& curr = _autofixState.output[idx];
    AutofixOutput& prev = _autofixState.outputPrevConv[idx];
    CameraInfo::CameraInstance inst = (CameraInfo::CameraInstance)idx;
    AutofixLimits &l = _autofixState.limits[idx];

    if (globalFrameIndex == 0) {
      PixelL0_i foeL0;
      foeL0.X() = foeL0.Y() = 0;
      curr.update(inst, _data.camera.invFocalLm2, foeL0);
      prev = curr;
      WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintValue(idx, "setAutofixOutput", "init"));
      return;
    }

    WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintConv(idx));
    bool conv[2] = {false, false};
    for (int i=0; i<2; ++i) {
      conv[i] = (_variable[i].conv() && globalFrameIndex - _data.algo.single[i].convLastFrame > 1);
      if (conv[i]) {
        curr.foeAngle[i] = _data.results.cams[idx].angles[i];
      }
    }

    if (!conv[e_YAW] && !conv[e_HORIZON]) {
      WmbcOutputIF::instance().toItrkAutofix(idx, l.foeL0Init[0]+l.foeAutofixL0Init[0]+curr.foeL0[0],
                                             l.foeL0Init[1]+l.foeAutofixL0Init[1]+curr.foeL0[1],
                                             _variable[e_YAW].conv(), _variable[e_HORIZON].conv(),
                                             &_autofixState, &_data);
      return;
    }

    curr.update(inst, _data.camera.focalLm2, curr.foeAngle);
    WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintValue(idx, "setAutofixOutput", "conv"));

    verifyDeltaL0Limits(idx); // foe value (px and angle) may change inside.
    if (!l.foeOutOfLimit[e_YAW] && !l.foeOutOfLimit[e_HORIZON]) {
      chopByAngle(idx, conv); // foe value (px and angle) may change inside.
    }

    for (int i=0; i<2; ++i) {
      if (conv[i]) {
        prev.foeAngle[i] = curr.foeAngle[i];
        prev.foeL0[i] = curr.foeL0[i];
      }
    }

    WmbcOutputIF::instance().toItrkAutofix(idx, l.foeL0Init[0]+l.foeAutofixL0Init[0]+curr.foeL0[0],
                                           l.foeL0Init[1]+l.foeAutofixL0Init[1]+curr.foeL0[1],
                                           _variable[e_YAW].conv(), _variable[e_HORIZON].conv(),
                                           &_autofixState, &_data);
  }

  void FOEFinder_Autofix::verifyDeltaL0Limits(int idx) {
    // Out-of-calib FailSafe is mimicking traditional autofix: exceeding 40px box around init calib
    // TODO: check again compatibility with old autofix

    AutofixState& s = _autofixState;
    AutofixLimits &l = s.limits[idx];
    AutofixOutput& curr = s.output[idx];
    CameraInfo::CameraInstance inst = (CameraInfo::CameraInstance)idx;
    PixelL0_i newFoe;
    newFoe.X() = newFoe.Y() = 0;

    for (int i=0; i<2; ++i) {
      int& deltaFoe = curr.foeL0[i];
      // current is ok
      float foeFixMagnitude = me_fabs(deltaFoe - l.foeAutofixDeltaL0LastUpdated[i]);
      if (foeFixMagnitude >= l.currIsOkThreshold) {
        l.foeCurrIsOk[i] = false;
      } else if (_variable[i].conv()) {
        l.foeCurrIsOk[i] = true;
      }

      WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintCurrIsOk(idx, i, foeFixMagnitude));

      // out of limits failsafe
      int initFoe = l.foeL0Init[i] + l.foeAutofixL0Init[i];
      newFoe[i] = initFoe + deltaFoe;

      if (_variable[i].conv()) {
        if (newFoe[i] > l.foeUpperLimitL0[i]) {
          deltaFoe = l.foeUpperLimitL0[i] - initFoe;
          l.foeOutOfLimit[i] = true;
        } else if (newFoe[i] < l.foeLowerLimitL0[i]) {
          deltaFoe = l.foeLowerLimitL0[i] - initFoe;
          l.foeOutOfLimit[i] = true;
        } else {
          l.foeOutOfLimit[i] = false;
        }
      }
    }

    if (l.foeOutOfLimit[e_YAW] || l.foeOutOfLimit[e_HORIZON]) {
      curr.update(inst, _data.camera.invFocalLm2, curr.foeL0);
      WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintValue(idx, "verifyDeltaL0Limits", "OOL! chopped"));
    }
    WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintOutOfLimit(idx, newFoe));
  }

  void FOEFinder_Autofix::chopByAngle(int idx, bool conv[2]) {
    AutofixOutput& curr = _autofixState.output[idx];
    AutofixOutput& prev = _autofixState.outputPrevConv[idx];
    CameraInfo::CameraInstance inst = (CameraInfo::CameraInstance)idx;

    bool toChop[2] = {false, false};
    for (int i=0; i<2; ++i) {
      if (conv[i]) {
        float dAngle = curr.foeAngle[i] - prev.foeAngle[i];
        float maxAngle = _data.metaParams.afixClippedJump[i];
        toChop[i] = (maxAngle > 0) && (me_fabsf(dAngle) > maxAngle);
        WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintToChop(idx, i, dAngle, toChop[i]));
        if (toChop[i]) {
          float sgn = (dAngle >= 0) ? 1.f : -1.f;
          WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintChopped(idx, i, dAngle, sgn));

          dAngle = sgn*_data.metaParams.afixClippedJump[i];
          curr.foeAngle[i] = prev.foeAngle[i] + dAngle;
          WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintChopResult(idx, i, dAngle));
        }
      }
    }
    if (toChop[e_YAW] || toChop[e_HORIZON]) {
      curr.update(inst, _data.camera.focalLm2, curr.foeAngle);
      WMBC_CALL(WmbcDbgStg::e_AUTOFIX, debugPrintValue(idx, "setAutofixOutput", "chopped"));
    }
  }

  void FOEFinder_Autofix::fillModelIF() {
    fillWmbcIF();
    fillAutoFixIF();

    bool conv = false;
    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      conv = conv || _data.algo.single[i].conv;
    }
    if (conv || globalFrameIndex <= 0) {
      Float::MEmath::Vec<3,float> T = Float::MEmath::zeros<3, float>();
      T[1] = -_data.results.curr.camh;
      Float::MEmath::Mat<3,3,float> R = matd2f(_data.results.curr.R);
      CameraToCameraAPI::setC2W_RT(R, T, CameraToCameraTypes::e_WMBC);
    }
  }

  void FOEFinder_Autofix::fillAutoFixIF() {
    MultiCameraAutoFixIF *m = &_autoFixIF.editable();
    const WmbcIF& mr = _wmbcIF.getObj();

    for (int i = 0; i < CameraInfo::e_NUM_OF_CAM_INSTANCES; ++i) {
      WMBC_CAM_SKIP(i)
      AutoFixIF* mi = &m->multiAutoFix[i];
      if (!mi->wasInitialized) {
        continue;
      }

      mi->autoFix_yawDelta = _autofixState.output[i].foeL0[e_YAW];
      mi->autoFix_horizonDelta = _autofixState.output[i].foeL0[e_HORIZON];
      mi->autoFix_yaw = mr.autoFix_yaw[i];
      mi->autoFix_horizon = mr.autoFix_horizon[i];

      mi->currentCalibrationIsOK_yaw = _autofixState.foeCurrIsOk[e_YAW];
      mi->currentCalibrationIsOK_horizon = _autofixState.foeCurrIsOk[e_HORIZON];

      mi->autofixPending = false;
      mi->pendingYawFix = 0;
      mi->pendingHorizonFix = 0;

      mi->progressCounter_yaw = mr.singleProgress[e_YAW];
      mi->progressCounter_horizon = mr.singleProgress[e_HORIZON];

      mi->yawProgress = mr.singleProgress[e_YAW];
      mi->horizonProgress = mr.singleProgress[e_HORIZON];

      // if temp autofix result is ready, but we're not updating to brain yet, we report the result here.
      // these fields are only for reporting, and do not affect brain/FFS
      if (_data.algo.single[e_YAW].conv) {
        mi->autofixPending = true;
        mi->pendingYawFix = _autofixState.output[i].foeL0[e_YAW];
      }
      if (_data.algo.single[e_HORIZON].conv) {
        mi->autofixPending = true;
        mi->pendingHorizonFix = _autofixState.output[i].foeL0[e_HORIZON];
      }
    }

    _autoFixIF.update();
  }

  void FOEFinder_Autofix::fillAutofixFailSafe(bool &autoFixFailSafe, bool &autoFixFailSafeYaw, bool &autoFixFailSafeHorizon) {
    autoFixFailSafeYaw = _autofixState.foeOutOfLimit[e_YAW];
    autoFixFailSafeHorizon = _autofixState.foeOutOfLimit[e_HORIZON];
    autoFixFailSafe = autoFixFailSafeYaw || autoFixFailSafeHorizon;
  }

  void FOEFinder_Autofix::updateReset() {
    int &m = _data.algo.resetMask;
    FOEFinder::updateReset();
    if (m < 0) { // debug skip reset
      return;
    }

    for (int i = 0; i < e_CALIB_DOF_NUM; ++i) {
      if (_data.algo.single[i].isSessionEnded()) {
        WMBC_PRINT(WmbcDbgStg::e_STATUS, WmbcDbgClr::e_BLUE, "[updateReset] reset %s - conv/timeout",
                   WMBC_DOF_ID_STR(i));
        m |= (1<<(6+i));
        reset(i);
      }
    }
  }

  // ------------------------------------------- AutofixOutput -----------------------------------------------------------
  AutofixOutput::AutofixOutput(const AutofixOutput& a) {
    for (int i=0; i<2; ++i) {
      foeL0[i] = a.foeL0[i];
      foeAngle[i] = a.foeAngle[i];
    }
  }

  AutofixOutput& AutofixOutput::operator=(const AutofixOutput& rhs) {
    for (int i=0; i<2; ++i) {
      foeL0[i] = rhs.foeL0[i];
      foeAngle[i] = rhs.foeAngle[i];
    }
    return *this;
  }

  AutofixOutput& AutofixOutput::operator+=(const AutofixOutput& that) {
    // TODO: should sum only angles and use update func?
    for (int i=0; i<2; ++i) {
      foeL0[i] += that.foeL0[i];
      foeAngle[i] += that.foeAngle[i];
    }
    return *this;
  }

  AutofixOutput& AutofixOutput::operator-=(const AutofixOutput& that) {
    // TODO: should sum only angles and use update func?
    for (int i=0; i<2; ++i) {
      foeL0[i] -= that.foeL0[i];
      foeAngle[i] -= that.foeAngle[i];
    }
    return *this;
  }

  void AutofixOutput::update(CameraInfo::CameraInstance inst, float invFocalLm2, const PixelL0_i foeL0In) {
    for (int i=0; i<2; ++i) {
      foeL0[i] = foeL0In[i];
    }

    bool ok = DistortionCorrectionAPI::isDistortionValid(inst);
    float x = foeL0[e_YAW]*4.f;
    float y = foeL0[e_HORIZON]*4.f;
    if (ok) {
      ok = DistortionCorrectionAPI::rectifySafe(inst, -2, x, y, foeAngle[e_YAW], foeAngle[e_HORIZON]);
    }
    if (ok) {
      foeAngle[e_YAW] *= invFocalLm2;
      foeAngle[e_HORIZON] *= invFocalLm2;
    } else {
      // TODO: assert and raise error
      foeAngle[e_YAW] = x*invFocalLm2;
      foeAngle[e_HORIZON] = y*invFocalLm2;
    }
  }

  void AutofixOutput::update(CameraInfo::CameraInstance inst, float focalLm2, const float foeAngleIn[2]) {
    for (int i=0; i<2; ++i) {
      foeAngle[i] = foeAngleIn[i];
    }

    bool ok = DistortionCorrectionAPI::isDistortionValid(inst);
    float yawLm2, horizonLm2;
    float x = foeAngle[e_YAW]*focalLm2;
    float y = foeAngle[e_HORIZON]*focalLm2;
    if (ok) {
      ok = DistortionCorrectionAPI::unrectifySafe(inst, -2, x, y, yawLm2, horizonLm2);
    }
    if (ok) {
      foeL0[e_YAW] = me_roundf(0.25*yawLm2); // TODO: align level-2 to level0 with rest of operation
      foeL0[e_HORIZON] = me_roundf(0.25*horizonLm2);
    } else {
      // TODO: assert and raise error
      foeL0[e_YAW] = me_roundf(0.25*x);
      foeL0[e_HORIZON] = me_roundf(0.25*y);
    }
  }

  AutofixOutput operator+(const AutofixOutput& a0, const AutofixOutput& a1) {
    AutofixOutput a = a0;
    a += a1;
    return a;
  }

  AutofixOutput operator-(const AutofixOutput& a0, const AutofixOutput& a1) {
    AutofixOutput a = a0;
    a -= a1;
    return a;
  }

  // ------------------------------------------- Prints  -----------------------------------------------------------
  // TODO: move printing functions to outputIF
  void FOEFinder_Autofix::debugPrintValue(int cidx, MEtl::string func, MEtl::string s) {
    if (cidx > 0) {
      return;
    }
    WMBC_PRINT(WmbcDbgStg::e_AUTOFIX, WmbcDbgClr::e_CYAN, "[%s] %s to: (%.2f, %.2f)deg = (%d, %d)px",
               func.c_str(), s.c_str(),
               _autofixState.output[cidx].foeAngle[e_YAW]*RAD2DEG,
               _autofixState.output[cidx].foeAngle[e_HORIZON]*RAD2DEG,
               _autofixState.output[cidx].foeL0[e_YAW],
               _autofixState.output[cidx].foeL0[e_HORIZON]);
  }

  void FOEFinder_Autofix::debugPrintConv(int cidx) {
    if (cidx > 0) {
      return;
    }
    for (int i=0; i<2; ++i) {
      WMBC_PRINT(WmbcDbgStg::e_AUTOFIX, WmbcDbgClr::e_CYAN, "[setAutofixOutput] %s: conv=%d, gfi-clf=%d-%d=%d",
                 (i==e_YAW ? "yaw" : "horizon"),
                 _variable[i].conv(), globalFrameIndex, _data.algo.single[i].convLastFrame,
                 globalFrameIndex - _data.algo.single[i].convLastFrame);
    }
  }

  void FOEFinder_Autofix::debugPrintToChop(int cidx, int vidx, float dAngle, bool toChop) {
    if (cidx > 0) {
      return;
    }
    WMBC_PRINT(WmbcDbgStg::e_AUTOFIX, WmbcDbgClr::e_CYAN, "[setAutofixOutput] %s: dAng=%.2f-%.2f=%.2f, "
               "toChop=%d",
               (vidx==e_YAW ? "yaw" : "horizon"),
               _autofixState.output[cidx].foeAngle[vidx]*RAD2DEG,
               _autofixState.outputPrevConv[cidx].foeAngle[vidx]*RAD2DEG,
               dAngle*RAD2DEG, toChop);
  }

  void FOEFinder_Autofix::debugPrintChopped(int cidx, int vidx, float dAngle, float sgn) {
    if (cidx > 0) {
      return;
    }
    WMBC_PRINT(WmbcDbgStg::e_AUTOFIX, WmbcDbgClr::e_CYAN, "[setAutofixOutput] %sDelta chopped: %.2f -> %.2f",
               (vidx==e_YAW ? "yaw" : "horizon"),
               dAngle*RAD2DEG,
               sgn*_data.metaParams.afixClippedJump[vidx]*RAD2DEG);
  }

  void FOEFinder_Autofix::debugPrintChopResult(int cidx, int vidx, float dAngle) {
    if (cidx > 0) {
      return;
    }
    WMBC_PRINT(WmbcDbgStg::e_AUTOFIX, WmbcDbgClr::e_CYAN, "[setAutofixOutput] "
               "%s-pp = prev+delta = %.2f+%.2f = %.2f",
               (vidx==e_YAW ? "yaw" : "horizon"),
               _autofixState.outputPrevConv[cidx].foeAngle[vidx]*RAD2DEG,
               dAngle*RAD2DEG,
               _autofixState.output[cidx].foeAngle[vidx]*RAD2DEG);
  }

  void FOEFinder_Autofix::debugPrintCurrIsOk(int cidx, int vidx, float foeFixMagnitude) {
    if (cidx > 0) {
      return;
    }
    WMBC_PRINT(WmbcDbgStg::e_AUTOFIX, WmbcDbgClr::e_BROWN, "[verifyDeltaL0Limits] "
               "%sFixMagnitude = |%d - %d| = %.0f, currIsOkThreshold=%.2f, %sCurrIsOk=%s",
               (vidx==e_YAW ? "yaw" : "horizon"),
               _autofixState.output[cidx].foeL0[vidx],
               _autofixState.limits[cidx].foeAutofixDeltaL0LastUpdated[vidx],
               foeFixMagnitude,
               _autofixState.limits[cidx].currIsOkThreshold,
               (vidx==e_YAW ? "yaw" : "horizon"),
               (_autofixState.limits[cidx].foeCurrIsOk[vidx] ? "yes" : "no"));
  }

  void FOEFinder_Autofix::debugPrintOutOfLimit(int cidx, PixelL0_i newFoe) {
    if (cidx > 0) {
      return;
    }
    for (int i=0; i<2; ++i) {
      WMBC_PRINT(WmbcDbgStg::e_AUTOFIX, WmbcDbgClr::e_BROWN, "[verifyDeltaL0Limits] new%s = %d + %d + %d = %d, "
                 "limits: (%d, %d), %sOutOfLimit=%s",
                 (i==e_YAW ? "Yaw" : "Horizon"),
                 _autofixState.limits[cidx].foeL0Init[i],
                 _autofixState.limits[cidx].foeL0Init[i],
                 _autofixState.output[cidx].foeL0[i],
                 newFoe[i],
                 _autofixState.limits[cidx].foeLowerLimitL0[i],
                 _autofixState.limits[cidx].foeUpperLimitL0[i],
                 (i==e_YAW ? "yaw" : "horizon"),
                 (_autofixState.limits[cidx].foeOutOfLimit[i] ? "yes" : "no"));

      WMBC_PRINT(WmbcDbgStg::e_AUTOFIX, WmbcDbgClr::e_CYAN, "[verifyDeltaL0Limits] %s: "
                 "autofixL0Init+Delta = %d + %d = %d vs (foeLm2-foeFull)/4 = (%d - %d)/4 = %d",
                 (i==e_YAW ? "yaw" : "horizon"),
                 _autofixState.limits[cidx].foeAutofixL0Init[i],
                 _autofixState.output[cidx].foeL0[i],
                 _autofixState.limits[cidx].foeAutofixL0Init[i] + _autofixState.output[cidx].foeL0[i],
                 (i==e_YAW ? _wmbcIF.getObj().yawLm2[CameraInfo::e_FORWARD] :
                  _wmbcIF.getObj().horizonLm2[CameraInfo::e_FORWARD]),
                 (i==e_YAW ? CameraInfo::yawFull(CameraInfo::e_FORWARD) :
                  CameraInfo::horizonFull(CameraInfo::e_FORWARD)),
                 (i==e_YAW ? (_wmbcIF.getObj().yawLm2[CameraInfo::e_FORWARD] -
                              CameraInfo::yawFull(CameraInfo::e_FORWARD))/4 :
                  (_wmbcIF.getObj().horizonLm2[CameraInfo::e_FORWARD] -
                   CameraInfo::horizonFull(CameraInfo::e_FORWARD))/4));
    }
  }

} // namespace WMBC
