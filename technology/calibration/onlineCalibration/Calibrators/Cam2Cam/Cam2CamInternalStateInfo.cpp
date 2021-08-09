#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamInternalStateInfo.h"
#include "technology/calibration/onlineCalibration/CalibUtils/calibMatUtils.h"
#include "utilities/clipextIO/clipextIO.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "basicTypes/MEtl/string.h"
#include <assert.h>

namespace OnlineCalibration
{
// class for sending debug info from the internal state of the C2C calibrator.
Cam2CamInternalStateInfo::Cam2CamInternalStateInfo() : _values {0}
{
    _readIndex = 0;
    _bHeadersReady = false;
    _headers = "";
    // we call setAllValues just to set the headers
    setAllValues(
        _translationWeight,
        _doFilterOutliers,
        _diffAnglesThreshold,
        _diffProjTransThreshold,
        _nInlierFrames,
        _outlierSigmas,
        _errorThreshold,
        _errorEstimate,
        _gammaI,
        _gammaII,
        _covErrorA,
        _covErrorB,
        _invCovError,
        _R0,
        _R,
        _t,
        _BAt,
        _M,
        _emFrameA,
        _emFrameB);
    _bHeadersReady = true;
}

void Cam2CamInternalStateInfo::getHeaders(MEtl::string &headers)
{
    headers = _headers;
}

void Cam2CamInternalStateInfo::setAllValues(
    float translationWeight,
    float doFilterOutliers,
    float diffAnglesThreshold,
    float diffProjTransThreshold,
    float nInlierFrames,
    float outlierSigmas,
    float errorThreshold,
    float errorEstimate,    
    float gammaI,
    float gammaII,
    Float::MEmath::Mat<3, 3, float> const &covErrorA,
    Float::MEmath::Mat<3, 3, float> const &covErrorB,
    Float::MEmath::Mat<3, 3, float> const &invCovError,
    Float::MEmath::Mat<3, 3, float> const &R0,
    Float::MEmath::Mat<3, 3, float> const &R,
    Float::MEmath::Vec<3, float> const &t,
    Float::MEmath::Mat<3, 3, float> const &BAt,
    Float::MEmath::Mat<3, 3, float> const &M,
    Float::MEmath::Mat<4, 4, float> const &emFrameA,
    Float::MEmath::Mat<4, 4, float> const &emFrameB)
{
    _writeIndex = 0;
    addScalar("translationWeight", translationWeight);
    addScalar("doFilterOutliers", doFilterOutliers);
    addScalar("diffAnglesThreshold", diffAnglesThreshold);
    addScalar("diffProjTransThreshold", diffProjTransThreshold);
    addScalar("nInlierFrames", nInlierFrames);
    addScalar("outlierSigmas", outlierSigmas);
    addScalar("errorThreshold", errorThreshold);
    addScalar("errorEstimate", errorEstimate);
    addScalar("gammaI", gammaI);
    addScalar("gammaII", gammaII);
    addMatrix("covErrorA", covErrorA);
    addMatrix("covErrorB", covErrorB);
    addMatrix("invCovError", invCovError);
    addMatrix("R0", R0);
    addMatrix("R", R);
    addMatrix("BAt", BAt);
    addMatrix("M", M);
    addVector("t", t);
    addMatrix("RA", CalibUtils::tform2rotm(emFrameA));
    addMatrix("RB", CalibUtils::tform2rotm(emFrameB));
    addVector("tA", CalibUtils::tform2trvec(emFrameA));
    addVector("tB", CalibUtils::tform2trvec(emFrameB));
}

void Cam2CamInternalStateInfo::addScalar(MEtl::string const &name, float value)
{
    assert(_writeIndex < _numOfValues);
    _values[_writeIndex++] = value;
    if (!_bHeadersReady)
    {
        _headers += " " + name;
    }
}

void Cam2CamInternalStateInfo::addVector(MEtl::string const& name, Float::MEmath::Vec<3, float> const& v)
{
    MEtl::string underscore("_");

    std::stringstream ss;
    for (int i = 0; i < 3; i++)
    {
        ss.str("");
        ss << i;
        addScalar(name + underscore + MEtl::string(ss.str().c_str()), v[i]);
    }
}

void Cam2CamInternalStateInfo::addMatrix(MEtl::string const& name, Float::MEmath::Mat<3, 3, float> const& A)
{
    MEtl::string underscore("_");
    std::stringstream ssi, ssj;
    for (int i = 0; i < 3; i++)
    {
        ssi.str("");
        ssi << i;
        for (int j = 0; j < 3; j++)
        {
            ssj.str("");
            ssj << j;
            addScalar(name + underscore + MEtl::string(ssi.str().c_str()) + MEtl::string(ssj.str().c_str()), A[i][j]);
        }
    }
}
} // namespace OnlineCalibration