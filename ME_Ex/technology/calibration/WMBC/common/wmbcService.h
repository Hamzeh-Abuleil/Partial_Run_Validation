
#ifndef WMBC_SERVICE_H_
#define WMBC_SERVICE_H_
#include "technology/calibration/WMBC/wmbcService_API.h"
class WmbcService:public WMBC::WmbcService_API{
  public:
    ~WmbcService()=default;
    // IF
    Fix::MEimage::Sync<WmbcIF>* getWmbcIF()  override;
    EWmbcIF* getEWmbcIF()  override;
    Fix::MEimage::Sync<WmbcIF>* getQuickWmbcIF()  override;

    // running mode status
    bool isRunning()  override;
    bool isRunningAutofix()  override;
    bool isRunningSPC()  override;
    bool isRunningSLC()  override;

    // confidence for REM
    float confidence()  override;
    int confidenceGrade()  override;

    // foe for WorldModel::CameraModel
    void getFoeInExpAllocated(CameraInfo::CameraInstance cinst, PrepSys::exp_mask exp, float& x, float& y, bool interim=false)  override;

    // autofix getters
    int  autoFix_yawDelta(CameraInfo::CameraInstance inst)  override;
    int  autoFix_horizonDelta(CameraInfo::CameraInstance inst)  override;
    bool yawConverged(CameraInfo::CameraInstance inst)  override;
    bool horizonConverged(CameraInfo::CameraInstance inst)  override;
    Fix::MEimage::Sync<MultiCameraAutoFixIF>* getAutoFixIF()  override;
    EMultiCameraAutoFixIF* getEAutoFixIF()  override;
};

#endif /* WMBC_SERVICE_H_ */
