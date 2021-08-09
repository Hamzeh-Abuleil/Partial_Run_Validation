#include "technology/calibration/onlineCalibration/CalibUtils/egomotionio.h"
#include "technology/calibration/onlineCalibration/CalibUtils/MatArray.h"
#include <cstdio>
#include <fstream>
#include <cassert>

namespace CalibUtils
{
    tformPairVec loadEgoMotion(std::string const &debugInDir)
    {
        if (debugInDir.empty())
        {
            return tformPairVec(0);
        }
        MatArray<4, 4, double> emSource, emTarget;
        assert(emSource.load(debugInDir + "/emSource.dat"));
        assert(emTarget.load(debugInDir + "/emTarget.dat"));
        size_t nFrames = emTarget.slices();
        assert(nFrames == emSource.slices());
        tformPairVec out(nFrames);
        for (size_t iFrame = 0; iFrame < nFrames; iFrame++)
        {
            out[iFrame].first = emSource.data[iFrame];
            out[iFrame].second = emTarget.data[iFrame];
        }
        return out;
    }
}