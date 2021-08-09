/**
 * \file  c2wOutputIF.cpp
 * \brief General constants
 *
 * \author Uri London
 * \date Jul 11, 2019
 */

#ifndef __OC_CAM2WORLD_CONSTS_IF_H_
#define __OC_CAM2WORLD_CONSTS_IF_H_

namespace OnlineCalibration {
  namespace Cam2World {
    const float RAD2DEG         = 57.29577951308232f;
    const float DEG2RAD         = 0.017453292519943295f;
    const float RAD2DEG_SQR     = RAD2DEG*RAD2DEG;
    const float DEG2RAD_SQR     = DEG2RAD*DEG2RAD;
    const float MPS2KPH         = 3.6f;
    const float KPH2MPS         = 0.28f;
    const float EPS             = 1e-6;
    const int   L0_TO_LM2_SCALE = 4;

    const int HIST_BIN_NUM = 500;
    const float HIST_BIN_SIZE[4] = {0.01f*DEG2RAD, 0.01f*DEG2RAD, 0.01f*DEG2RAD, 0.001f}; // yaw [rad], hor [rad], roll [rad], camH [m]
    const int HIST_MAX_SIZE = 1E5;
    const int HIST_START_FRAME = 5;

    // stuff to do with calculation of correlation time:
    const int SF_BUFF_SIZE = 128;
    const int MIN_FRAMES_4_CORR = 16;
    const float LAG_PORTION = 0.8; // portion of window to consider for acoor calc


    const int VALID_WIN_SIZE = 32; // window size of valid counter, must be power of two (fastCyclicVector)
    const int UPDATE_WIDTH_DIFF = 8; // threshold of difference in correlation time to update kernel in convolver
    

    const int PLANE_BUFFER_CAPACITY = 8;
    const unsigned int CONVOLVE_WIN_SIZE = 64;

    const int SPC_SAMPLE_EXTRA   = 20;
    const int SPC_MIN_SAMPLE     = 100;
    const int SPC_MIN_STABLE     = 25;
    const float SPC_MAX_VAR_ANG  = 0.25f*DEG2RAD_SQR;
    const float SPC_MAX_VAR_DIST = 0.0025f;

    // moving centers
    enum MovingWindowSize {
      e_FAST,
      e_MEDIUM,
      e_SLOW,
      e_WIN_SIZE_NUM
    };
    const float EMA_FACTOR[e_WIN_SIZE_NUM] = {0.9f, 0.97f, 0.99f};
    const unsigned int HIST_WIN_SIZE[e_WIN_SIZE_NUM] = {8, 64, 256}; // window size of moving averages/median, must be power of two

    enum C2WSig {
      e_YAW,
      e_PITCH,
      e_ROLL,
      e_CAM_HEIGHT,
      e_C2W_SIG_NUM
    };

#ifdef MEwin
    const MEtl::string s_C2WSig[e_C2W_SIG_NUM] = {"e_YAW", "e_PITCH", "e_ROLL", "e_CAM_HEIGHT"};
#endif

    enum NonValidBits {
      e_SENSORS_UNAVAILABLE          = 1,
      e_SPEED_TOO_LOW                = 2,
      e_SPEED_TOO_HIGH               = 4,
      e_YAWRATE_TOO_HIGH             = 8,
      e_RADIUS_TOO_SMALL             = 16,
      e_ACCELERATION_TOO_HIGH        = 32,
      e_EM_INVALID                   = 64,
      e_PLANE_INVALID                = 128,
      e_RM_INVALID                   = 256,
      e_EM_STRAIGHT                  = 512,
      e_EM_RM_PITCHDIFF              = 1024,
      e_ROAD_CROWN                   = 2048,
      e_FRAME_VALID_COND_NUM         = 12
    };

    const int VEHICLE_INVALID = (e_SENSORS_UNAVAILABLE |
                                 e_SPEED_TOO_LOW |
                                 e_SPEED_TOO_HIGH |
                                 e_YAWRATE_TOO_HIGH |
                                 e_RADIUS_TOO_SMALL |
                                 e_ACCELERATION_TOO_HIGH
                                );

    const int FOE_INVALID = (VEHICLE_INVALID |
                             e_EM_INVALID |
                             e_EM_STRAIGHT
                            );

    const int ROLL_INVALID = (FOE_INVALID |
                              e_PLANE_INVALID
                             );

    const int CAMH_INVALID = ROLL_INVALID;

    const int INVALID_MASK[e_C2W_SIG_NUM] = {FOE_INVALID, FOE_INVALID, ROLL_INVALID, CAMH_INVALID};

    enum ScalingModes {
      e_SCALE_LINEAR,
      e_SCALE_POW,
      e_SCALE_EXP,
      e_SCALE_SIGMOID,
      e_SCALE_MODE_NUM
    };

  } // namespace Cam2World
} // namespace OnlineCalibration

#endif // __OC_CAM2WORLD_CONSTS_IF_H_
