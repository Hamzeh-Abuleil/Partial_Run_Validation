/**
 * \file SteadyStateCalibrator.h
 * \brief SteadyStateCalibrator header
 *
 * \author Uri London
 * \date Jul 11, 2019
 */


#ifndef __OC_CAM2wORLD_STEADY_STATE_H_
#define __OC_CAM2wORLD_STEADY_STATE_H_

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/Cam2WorldSources.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wSignal.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/planeSync.h"

namespace OnlineCalibration {
  namespace Cam2World {

    class SteadyStateCalibrator {
      public:
        SteadyStateCalibrator();

        void update(const Cam2WorldSources& source);
        void computeInit(const Cam2WorldSources& source);
        void compute();
        void stopSPC() { _data.spc.stopped = true;}
        void restartSPC() { _data.spc.reset(); reset();}
        void setCameraData(const Cam2WorldSources& source);
        void setExtraParams(const Cam2WorldSources& source);

        Float::MEmath::Mat<3, 3, float>   rotation() const { return _data.results.R;}
        Float::MEmath::Vec<3, float>  translation() const { return _data.results.t;}
        //float confidence() const { return _data.results.confidence;}
        unsigned int confidence() const { return _data.results.confidence;}
        unsigned int stringentConfidence() const { return _data.results.stringentConfidence;}
        bool isCalibInRange() const;
        bool isSignalStable() const;
        const ResultBook& results() const { return _data.results;}
        const SpcData& spcData() const { return _data.spc;}
        int degradeCause() const { return _data.algo.degradeCause;}

        int getEmState() { return _data.em.status; }
        int getRmState() { return _data.rm.status; }

      private:
        void reset();
        void setParams();
        void updateReset();
        void setInputData(const Cam2WorldSources& source);
        void setVehicleData(const Cam2WorldSources& source);
        void setWmEgomotionData(const Cam2WorldSources& source);
        void setWmRoadModelData(const Cam2WorldSources& source);
        void setWmConfData(const Cam2WorldSources& source);
        void setWmRoadModelCov(const WorldModel::VRM::SegmentModel* model,
                               const WorldModel::EgoMotion::RoadModelStorage *storage);
        void updateFoeConfidence(TrackingConfidenceData &c, int level);
        void setVarOfSignals();
        void propagatePlaneUnderCam();
        void validateFrameVehicle();
        void validateFrameAlgo();
        bool isStraightDrive();
        void computeRotationTranslation();
        void computeConfidence();
        bool spcEnded();
        void spcUpdateValidFrame();
        void spcUpdateStatus();
        void spcUpdateReset();
        void spcRestartSession();
        void spcUpdateFoe();
        void printDebugWmCsv(const WorldModel::VRM::SegmentModel *model,
                             const WorldModel::EgoMotion::RoadModelStorage *storage,
                             TrackingConfidenceData &cdata, int idx);


        Cam2WorldData _data;
        PlaneSync _planeSync;
        Cam2WorldSignal _signal[e_C2W_SIG_NUM];
        // int _stableCountThresh;
    };

  } // Cam2World
} // OnlineCalibration

# endif //  __OC_CAM2wORLD_STEADY_STATE_H_
