#pragma once
#include <vector>
#include <string>
#include <utility>
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"

namespace CalibUtils
{
    // To load an EgoMotion pair from file, debugInDir should
    // include two files called emSource.dat and emTarget.dat
    // These files should be formatted as in MatArray.h
    tformPairVec loadEgoMotion(std::string const &debugInDir);
}