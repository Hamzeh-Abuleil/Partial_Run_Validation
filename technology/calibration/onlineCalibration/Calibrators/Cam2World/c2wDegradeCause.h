/**
 * \file c2wDegradeCause.h
 * \brief DegradeCause header
 *
 * \author Uri London
 * \date May 11, 2020
 */


#ifndef __OC_CAM2WORLD_DCAUSE_H
#define __OC_CAM2WORLD_DCAUSE_H

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wConsts.h"
#include "basicTypes/containers/fastCyclicVector.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationDefs.h"

namespace OnlineCalibration {
  namespace Cam2World {

  const float DEGRADE_CONF_TH = 0.7f;
  const int DEGRADE_FRAME_VALID = ((int)StateDegradeCause::EM_INVALID |
                                   (int)StateDegradeCause::RM_INVALID |
                                   (int)StateDegradeCause::TIMEOUT);

  struct DegradeFrameProperties {
    DegradeFrameProperties() : emValid(false), rmValid(false), stable(false),
                               highConf(false), validFrame(false) {}

    bool emValid;
    bool rmValid;
    bool stable;
    bool highConf;
    bool validFrame;
  };

  class Cam2WorldSignalDegradeReporter {
    public:
      Cam2WorldSignalDegradeReporter();
      ~Cam2WorldSignalDegradeReporter() {}

      void reset();
      void update(int invalidMask);
      void update(float conf, bool stability);

      int mask() const { return _mask;}

    private:
      void update();
      int _mask;
      MEtypes::FastCyclicVector<DegradeFrameProperties> _buff; 
  };

  } // Cam2World
} // OnlineCalibration

#endif // __OC_CAM2WORLD_DCAUSE_H

