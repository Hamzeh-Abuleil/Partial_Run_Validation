
#ifndef WMBC_SERVICE_API_H_
#define WMBC_SERVICE_API_H_

#include "technology/calibration/WMBC/interface/wmbcIF.h"
#include "functionality/interface/autoFixIF.h"
#include "technology/brain2/prepSys/prepSys_API.h"
#include "functionality/ServiceLocator/ServiceLocator_API.h"
namespace Fix {
  namespace MEimage {
    template < class T > class Sync;
  }
}


namespace WMBC {

  struct TargetInfo;
  class WmbcService_API:public ServiceLocator_API::APIProvider{
    public:
      virtual ~WmbcService_API()=default;
      // IF
      virtual Fix::MEimage::Sync<WmbcIF>* getWmbcIF()=0;
      virtual EWmbcIF* getEWmbcIF()=0;
      virtual Fix::MEimage::Sync<WmbcIF>* getQuickWmbcIF()=0;
  
      // running mode status
      virtual bool isRunning()=0;
      virtual bool isRunningAutofix()=0;
      virtual bool isRunningSPC()=0;
      virtual bool isRunningSLC()=0;

      // confidence for REM
      virtual float confidence()=0;
      virtual int confidenceGrade()=0;

      // foe for WorldModel::CameraModel
      virtual void getFoeInExpAllocated(CameraInfo::CameraInstance cinst, PrepSys::exp_mask exp, float& x, float& y, bool interim=false)=0;

      // autofix getters
      virtual int  autoFix_yawDelta(CameraInfo::CameraInstance inst)=0;
      virtual int  autoFix_horizonDelta(CameraInfo::CameraInstance inst)=0;
      virtual bool yawConverged(CameraInfo::CameraInstance inst)=0;
      virtual bool horizonConverged(CameraInfo::CameraInstance inst)=0;
      virtual Fix::MEimage::Sync<MultiCameraAutoFixIF>* getAutoFixIF()=0;
      virtual EMultiCameraAutoFixIF* getEAutoFixIF()=0;
      static std::string name(){return "WMBC";}
  };
}
#endif /* WMBC_SERVICE_API_H_ */
