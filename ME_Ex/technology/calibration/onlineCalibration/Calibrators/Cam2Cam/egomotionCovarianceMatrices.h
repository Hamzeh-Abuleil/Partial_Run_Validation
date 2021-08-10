/**
 * \file egomotionCovarianceMatrices.h
 * \brief Returns hard-coded error covariance matrices for various camera types.
 * 
 * \author Amit Hochman
 * \date Jan 14, 2019
 */
#pragma once
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationCommonDefs.h"

namespace EgomotionCalibration
{
CalibUtils::mat33 egomotionCovarianceMatrix(OnlineCalibration::CoordSys coordSys);
}