#pragma once

#include "utilities/clipextIO/clipextIO.h"
#include "technology/mobilib/float/common/MEmath/mat.h"
#include "basicTypes/MEtl/string.h"

namespace OnlineCalibration
{
// class for sending debug info from the internal state of the C2C calibrator.
class Cam2CamInternalStateInfo
{
public:
    Cam2CamInternalStateInfo();
    void setAllValues(
        float translationWeight,      //< Relative weight for translation equations.
        float doFilterOutliers,       //< Set to true to filter outliers
        float diffAnglesThreshold,    //< Angle difference above which data is considered an outlier
        float diffProjTransThreshold, //< Difference in projected translation above which data is considered an outlier
        float nInlierFrames,          //< Number of frames that have been processed so far.
        float outlierSigmas,          //< range in standard-deviations beyond which a datum is considered an outlier
        float errorThreshold,         //< Below this we trust R enough to use it to filter outliers (in degrees)
        float errorEstimate,          //< estimate of total error in rotation, in degrees.
        float gammaI,
        float gammaII,
        Float::MEmath::Mat<3, 3, float> const &covErrorA,   //< covariance of errors in 'A' inputs, as in R*A \approx B
        Float::MEmath::Mat<3, 3, float> const &covErrorB,   //< covariance of errors in 'B' inputs
        Float::MEmath::Mat<3, 3, float> const &invCovError, //< inverse of covariance matrix used in error estimate
        Float::MEmath::Mat<3, 3, float> const &R0,          //< Procrustes-based initial guess for nonlinear solver
        Float::MEmath::Mat<3, 3, float> const &R,           //< Current estimate for rotation part of calibration
        Float::MEmath::Vec<3, float> const &t,              //< Current estimate for translation vector
        Float::MEmath::Mat<3, 3, float> const &BAt,
        Float::MEmath::Mat<3, 3, float> const &M,
        Float::MEmath::Mat<4, 4, float> const &emFrameA,
        Float::MEmath::Mat<4, 4, float> const &emFrameB);
    const float *getValues() const { return _values; };
    const unsigned int getNumOfValues() const { return _numOfValues; };
    void getHeaders(MEtl::string &headers);

private:
    unsigned int _readIndex;
    unsigned int _writeIndex;
    MEtl::string _headers;
    static const unsigned int _numOfValues = 100;
    float _values[_numOfValues];
    void addMatrix(MEtl::string const &name, Float::MEmath::Mat<3, 3, float> const &A);
    void addVector(MEtl::string const &name, Float::MEmath::Vec<3, float> const &v);
    void addScalar(MEtl::string const &name, float value);
    bool _bHeadersReady;
    float _translationWeight;                     //< Relative weight for translation equations.
    float _doFilterOutliers;                      //< Set to true to filter outliers
    float _diffAnglesThreshold;                   //< Angle difference above which data is considered an outlier
    float _diffProjTransThreshold;                //< Difference in projected translation above which data is considered an outlier
    Float::MEmath::Mat<3, 3, float> _R0;          //< Procrustes-based initial guess for nonlinear solver
    Float::MEmath::Mat<3, 3, float> _R;           //< Current estimate for rotation part of calibration
    Float::MEmath::Vec<3, float> _t;              //< Current estimate for translation vector
    float _nInlierFrames;                         //< Number of frames that have been processed so far.
    float _outlierSigmas;                         //< range in standard-deviations beyond which a datum is considered an outlier
    float _errorThreshold;                        //< Below this we trust R enough to use it to filter outliers (in degrees)
    float _errorEstimate;                         //< estimate of total error in rotation, in degrees.
    Float::MEmath::Mat<3, 3, float> _covErrorA;   //< covariance of errors in 'A' inputs, as in R*A \approx B
    Float::MEmath::Mat<3, 3, float> _covErrorB;   //< covariance of errors in 'B' inputs
    Float::MEmath::Mat<3, 3, float> _invCovError; //< inverse of covariance matrix used in error estimate
    float _bComputeErrorEstimate;                 //< we stop once we drop below errorThreshold
    // state matrices - see write-up for details:
    // http://portal.mobileye.com/kt/Technologies/Documents/calibration_/Online%20Calibration/Cam2Cam/algorithm_write_up.pdf?Web=1
    Float::MEmath::Mat<3, 3, float> _BAt, _M;
    float _gammaI, _gammaII;
    Float::MEmath::Mat<4, 4, float> _emFrameA, _emFrameB; // the EM inputs, for debug purposes
};
} // namespace OnlineCalibration