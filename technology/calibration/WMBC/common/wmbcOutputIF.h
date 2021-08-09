/*
 * wmbcOutputIF.h
 *
 *  Created on: Jun 08, 2016
 *      Author: urilo
 */

#ifndef _WMBC_OUTPUT_IF_
#define _WMBC_OUTPUT_IF_

#include "utilities/cameraInformation/cameraInformation_API.h"
#include "technology/calibration/WMBC/common/slcTypes.h"
#include "technology/mobilib/fix/common/MEXimage/sync.h"
#include "technology/mobilib/fix/common/MEXimage/typeImages.h"
#include "basicTypes/containers/fastCyclicVector.h"

struct WmbcIF;

namespace itrkWriter {
  class ItrkWriterHandle;
}

namespace WMBC {
  // struct WorldModelData;
  // struct VehicleData;
  // struct ValidationParams;
  class CalibVariable;
  class HistogramOneDim;
  class PlaneSync;
  struct Plane;
  struct CamhData;
  struct AutofixState;

  struct WmbcItrkHandleNames {
    enum ItrkType {e_INPUT_DATA, 
                   e_INPUT_DATA_CONF,
                   e_FRAME_VALIDATION, 
                   e_FOE, e_ROLL, 
                   e_CALIB_VAR,
                   e_HIST,
                   e_PLANE,
                   e_PLANE_BUFFER,
                   e_CONVERGENCE, 
                   e_SLC_TARGETS, 
                   e_SLC_CAMH, 
                   e_SLC_CONVERGENCE,
                   e_DEBUG,
                   e_FINAL, 
                   e_FPA, 
                   e_AFIX,
                   e_ITRK_NUM_TYPES};
  };
  typedef WmbcItrkHandleNames::ItrkType WmbcItrkType;

  class WmbcOutputIF {
  public:
    

    static WmbcOutputIF& instance();
    ~WmbcOutputIF() {}

    void toItrkHeaders();
    void toItrkInputData(const WmbcData *d) const;
    void toItrkFrameValidation(const WmbcData *d) const;
    void toItrkFoe(const HistogramOneDim *histYaw, const HistogramOneDim *histHorizon, float focalLm2, bool quickMode = false) const;
    void toItrkRoll(const HistogramOneDim *histRoll, const HistogramOneDim *histCamh, bool quickMode = false) const;
    void toItrkCalibVariable(const CalibVariable *c, const WmbcData* d) const;
    void toItrkPlane(const WmbcData *d) const;
    void toItrkPlaneBuffer(const MEtypes::FastCyclicVector<Plane> *p) const;
    void toItrkConvergence(int calibInd) const;
    void toItrkSlcTargets(bool valid, SPInfoVec *sp, float imHeight, float imWidth, float distToTargets, const TargetParams *tparams, 
                          const Fix::MEimage::Rect *targetSearchWindow, int targetsLevel) const;
    void toItrkSlcCamHeight(CamhDataVec *cd, SlcConvParams *cp, bool *phaseStates, float camhErrMean) const;
    void toItrkSlcConvergence(bool isEndTraj, int validFramesNum, const WmbcData *d, const SlcConvParams *cparams, const TargetParams *tparams) const;
    void toItrkDebug(const WmbcData *d) const;
    void toItrkFinal(const WmbcIF *wmbcIF, const WmbcData *d) const;
    void toItrkAutofix(int idx, int newYaw, int newHorizon, bool convYaw, bool convHorizon, const AutofixState *as, const WmbcData* d) const;

    void toCextTargetPts(SPInfoVec *sp, const Prep::SafeImg* img);

  private:
    WmbcOutputIF();

    itrkWriter::ItrkWriterHandle *_itrkHandle[WmbcItrkType::e_ITRK_NUM_TYPES];
    CameraInfo::CameraInstance _camInst;
  };

} // namespace WMBC

#endif // __WMBC_OUTPUT_IF__
