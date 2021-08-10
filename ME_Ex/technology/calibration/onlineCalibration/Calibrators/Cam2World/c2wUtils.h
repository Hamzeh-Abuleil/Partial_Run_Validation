/**
 * \file  c2wOutputIF.h
 * \brief Math Utilities
 *
 * \author Uri London
 * \date Jul 11, 2019
 */

#ifndef __OC_CAM2WORLD_UTILS_H_
#define __OC_CAM2WORLD_UTILS_H_

#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibration.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"


namespace OnlineCalibration {
  namespace Cam2World {

    Float::MEmath::Mat<3, 3, float> composeRotationMatrixFromSensorAngles(const float yaw, const float pitch, const float roll);
    Float::MEmath::Mat<3, 3, double> composeRotationMatrixFromSensorAngles(const double yaw, const double pitch, const double roll);
    void decomposeRotationMatrixIntoSensorAngles(const Float::MEmath::Mat<3, 3, float>, float& yaw, float& pitch, float& roll);
    void foeBoundariesFromImageCenter2PP(CalibUtils::PixelLm2 minFoe_con, CalibUtils::PixelLm2 maxFoe_con, CalibUtils::PixelLm2& minFoe_pp, CalibUtils::PixelLm2& maxFoe_pp);
    bool anglesToPixel(float yaw, float pitch, CalibUtils::PixelLm2_f& foe);
    bool pixelToAngles(const CalibUtils::PixelLm2_f foe, float& yaw, float& pitch);
    bool rotationMatrixToSensorPixel(const Float::MEmath::Mat<3, 3, float> R, CalibUtils::PixelLm2_f& foe, float& roll);
    bool rotationMatrixToRodriguezPixel(const Float::MEmath::Mat<3, 3, float> R, CalibUtils::PixelLm2_f& foe, float& roll);
    Float::MEmath::Mat<3, 3, double> matf2d(Float::MEmath::Mat<3, 3, float> matf);
    Float::MEmath::Mat<3, 3, float> matd2f(Float::MEmath::Mat<3, 3, double> matd);
    void matf2arr(const Float::MEmath::Mat<3, 3, float> matf, float arr[]);
    void vecf2arr(const Float::MEmath::Vec<3, float> vecf, float arr[]);
    void copyArr(int size, const float arrIn[], float arr[]);
  } // namespace Cam2World
} // namespace OnlineCalibration

#endif // __OC_CAM2WORLD_UTILS_H_

