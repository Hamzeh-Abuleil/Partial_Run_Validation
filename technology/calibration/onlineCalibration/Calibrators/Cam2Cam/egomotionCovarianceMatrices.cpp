/**
 * \file egomotionCovarianceMatrices.cpp
 * \brief Implementation of hard-coded error covariance matrices.
 *
 * \author Amit Hochman
 * \date Jan 14, 2019
 */
#include "egomotionCovarianceMatrices.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibMatUtils.h"
#include "functionality/calibration/cameraInformationProperTypes.h"

CalibUtils::mat33 EgomotionCalibration::egomotionCovarianceMatrix(OnlineCalibration::CoordSys coordSys)
{
    // At this point we only have two covariance matrices, one for rear, and one
    // for all the rest. In the future, we might use a switch statement here for
    // different camera types.
    double data[9];
    CameraInfo::CameraInstance camera = OnlineCalibration::coordsToInstance(coordSys);
    if (camera == CameraInfo::CameraInstance::e_REAR)
    {
        double rear[9] = {7.46713005e-09, -7.33746243e-12, -1.53463035e-09, -7.33746243e-12,
                                2.12879316e-08, 1.124082e-08, -1.53463035e-09, 1.124082e-08,
                                6.10683127e-08};
        std::memcpy(data, rear, sizeof(rear));
    }
    else
    {
        static const double forward[9] = {7.89629821e-09, -3.42769315e-10, -9.39628606e-10, -3.42769315e-10,
                                   2.05388002e-08, 5.65815911e-09, -9.39628606e-10, 5.65815911e-09,
                                   6.91799984e-08};
        std::memcpy(data, forward, sizeof(forward));
    }
    return CalibUtils::mat<3, 3>(data);
}