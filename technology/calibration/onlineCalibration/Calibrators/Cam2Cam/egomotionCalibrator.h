/**
 * \file egomotionCalibrator.h
 * \brief Egomotion calibrator class header.
 *
 * \author Amit Hochman
 * \date Dec 13, 2018
 */
#pragma once
#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"
#include "functionality/calibration/cameraInformationProperTypes.h"
#include "technology/calibration/onlineCalibration/Calibrators/Cam2Cam/Cam2CamInternalStateInfo.h"
#include "technology/calibration/onlineCalibration/OnlineCalibrationDefs.h"

namespace EgomotionCalibration
{

/**
 * EgomotionCalibrator is used to compute the extrinsic camera to camera calibration
 * based on EgoMotion of the two cameras. Each time a new pair of EgoMotion frames
 * becomes avaliable, a call to update() updates the state of the calibrator to reflect
 * the information in the two frames. Based on this state, compute() computes the
 * calibration and updates the internal _R and _t fields that hold the rotation
 * and translation parts of the calibration. Forming a 4x4 calibration matrix T from
 * _R and _t, we obtain approximately that, M*TA_i*M' = TB_i, where TA_i and TB_i are
 * the EgoMotion matrices, for all frames i that have been
 * added to the calibrator.
 */
class EgomotionCalibrator
{
  public:
  EgomotionCalibrator(CameraInfo::CameraInstance cameraA = CameraInfo::e_FORWARD,
                      CameraInfo::CameraInstance cameraB = CameraInfo::e_REAR)
    {
        reset();
    };

    /**
    * Update calibrator with data from a pair of EgoMotion frames, belonging to
    * different cameras at the same instant in time.
    * \param emFrameA EgoMotion frame A
    * \param emFrameB EgoMotion frame B
    */
    void update(CalibUtils::mat44 const &emFrameA, CalibUtils::mat44 const &emFrameB);

    /**
    * Compute calibration. Internal calibration, stored in _R and _t is updated.
    */
    void compute(void);

    /**
    * \return Current estimate for the rotation matrix.
    */
    CalibUtils::mat33 getRotation() const;
    
    /**
    * \return Current estimate for the dynamic rotation matrix.
    */
    CalibUtils::mat33 getDynamicRotation() const;
    
    /**
    * \return Current confidence
    */
    double getConfidence() const;

    /**
    * \return Current state
    */
    OnlineCalibration::State getState() const;
    
    /**
    * \return Current estimate for the translation vector.
    */
    CalibUtils::vec3 getTranslation() const;
    /**
    * Resets all state matrices to initial values.
    */
    void reset(void);

    /**
     * Sets covariance matrices to be used in computing the estimated covariance
     * of the calibrator's error
     * \param covarianceCameraA
     * \param covarianceCameraB
     */
    void setCovErrorMatrices(CalibUtils::mat33 const &covarianceCameraA, CalibUtils::mat33 const &covarianceCameraB);

    /**
     * Returns internal state info for output to itrk, mainly for debug purposes.
     * \return state info
     */
    const OnlineCalibration::Cam2CamInternalStateInfo getStateInfo() const;
  private:
    /////////////
    // Methods //
    /////////////
    void updateGuess(void); //< updates _R0, the initial guess for nonlinear solver
    void updateDynamic(void); //< updates _RDynamic, the dynamic calibration
    void updateConfidence(void); //< updates confidence based on difference from dynamic calib + errorEstimate
    void solveForR(void);   //< Does the nonlinear solve and updates _R
    /**
    * Filters outliers, based on the differences between the rotation angles,
    * and the difference between the projections of the translation on the rotation angle.
    * Both these quantities are supposed to be invariant to a change in coordinates, so if they
    * differ, that signals the data might be bad.
    * It also computes how well the current EM pair fits the current estimate of
    * the cam2cam R and t. If, based on _errorEstimate, we trust these R and t,
    * we may flag the EM pair as an outlier. If the _errorEstimate is still >
    * _errorThreshold, we may flag the EM pair as a suspect if it does not fit
    * the current R and t well.
    * \param a rotation axis (with norm = rotation angle) for frame A
    * \param b rotation axis (with norm = rotation angle) for frame B
    * \param emFrameA tform matrix for frame A
    * \param emFrameB tform matrix for frame B
    * \return true if data should be considered an outlier, false otherwise.
    */
    bool isOutlier(CalibUtils::vec3 const &a, CalibUtils::vec3 const &b, CalibUtils::mat44 const &emFrameA, CalibUtils::mat44 const &emFrameB);
    /**
     * Computes an update for the inverse of the covariance matrix of the error
     * in the rotation. Based on "Optimal estimation of three-dimensional
     * rotation and reliability evaluation"
     * \param b rotation axis of frame B
     * \return Matrix to add to _invCorError to keep it up-to-date.
     */
    CalibUtils::mat33 compute_inv_cov_error_update(CalibUtils::vec3 const &b) const;
    /**
     * Computes the RMS values of the residuals of the rotation and translation
     * equations (see write-up).
     */
    void compute_residuals(double &rmsR, double& rmsT) const;
    /**
     * Checks all potential outliers in _outlierSuspects, and removes the ones
     * that are found to be outliers.
     */
    void purgeOutliers(void);
    /**
     * Updates the _stateInfo variable, which is used to output debug information to an
     * itrk file. For debug purposes, we also add:
     * \param emFrameA tform matrix for frame A
     * \param emFrameB tform matrix for frame B
     */
    void updateStateInfo(CalibUtils::mat44 const &emFrameA, CalibUtils::mat44 const &emFrameB);
    //////////////////////
    // Member variables //
    //////////////////////
  const double _translationWeight = 0.01;      //< Relative weight for translation equations.
  const bool _doFilterOutliers = true;         //< Set to true to filter outliers
  const double _diffAnglesThreshold = 0.0004;  //< Angle difference above which data is considered an outlier
  //const double _diffProjTransThreshold = 0.07; //< Difference in projected translation above which data is considered an outlier
  const double _diffProjTransThreshold = 0.7; //< Difference in projected translation above which data is considered an outlier
  const double _forgetFactor = 0.97;           //< Multiply _BAtDynamic by this on update, to forget history gradually
  const double _errorThreshold = 0.25;         //< Below this we trust R enough to use it to filter outliers (in degrees)
  const double _outlierSigmas = 3.0;           //< range in standard-deviations beyond which a datum is considered an outlier

  // Confidence is the product of three factors: the errorEstimate, the dynamic outlier rate
  // and the difference between the static and dynamic rotation estimates.
  // Each factor is computed using a transfer-function that is 1 at 0 and 0 at and above
  // some upper limit. There is also a critical value where the transfer-functions
  // begins to decrease. All three factors are multiplied to get the confidence score.
  // See also: butter_weight

  const double _errorEstimateCritical = 0.3; //< in degrees
  const double _errorEstimateRoot = 3;
  const double _errorEstimateExp = 7;      //< Higher value -> flatter response and faster transition to decreasing region.
  const double _diffDynamicCritical = 0.5; //< degrees
  const double _diffDynamicRoot = 2;
  const double _diffDynamicExp = 10;
  const double _outlierRateCritical = 0.2;
  const double _outlierRateRoot = 0.6;
  const double _outlierRateExp = 20;
  const double _resetThreshold = 0.9; //< we reset if confidence factors (except static-dynamic difference) are above this and static-dynamic diff == 0
  double _dynamicOutliers;
  double _dynamicNumFrames;

  const double _stateThresholds[2] = {0.5, 0.8}; // less than 0.5 conf. means suspected, more than 0.85 mean good, etc.
  static const unsigned _nPastStates = 21;
  Float::MEmath::Vec<_nPastStates, OnlineCalibration::State> _pastStates;
  OnlineCalibration::State _state; // state will be median(pastStates)
  double _confidence;                  // confidence in accuracy of calibration

    CalibUtils::mat33 _R0;                //< Procrustes-based initial guess for nonlinear solver
    CalibUtils::mat33 _RDynamic; //< The dynamic calibration (i.e., that keeps track of potential changes in the calibration)
    CalibUtils::mat33 _R;                 //< Current estimate for rotation part of calibration
    CalibUtils::vec3 _t;                  //< Current estimate for translation vector
    unsigned int _nInlierFrames;          //< Number of frames that have been processed so far.
    struct SuspectData                    // For EM data we suspect to be an outlier
    {
        CalibUtils::vec3 a, b, ta, tb;
        CalibUtils::mat33 Rb;
    };
    std::vector<SuspectData> _outlierSuspects; //< Vector of data frames that might be outliers

    double _errorEstimate;                     //< estimate of total error in rotation, in degrees.
    CalibUtils::mat33 _covErrorA;              //< covariance of errors in 'A' inputs, as in R*A \approx B
    CalibUtils::mat33 _covErrorB;              //< covariance of errors in 'B' inputs
    CalibUtils::mat33 _invCovError;            //< inverse of covariance matrix used in error estimate
    bool _bComputeErrorEstimate;               //< we stop once we drop below errorThreshold
    // state matrices - see write-up for details:
    // http://portal.mobileye.com/kt/Technologies/Documents/calibration_/Online%20Calibration/Cam2Cam/algorithm_write_up.pdf?Web=1
    CalibUtils::mat33 _BAt, _BAtDynamic, _M;
    CalibUtils::mat99 _A1tA1, _A2tA2, _K;
    CalibUtils::vec3 _V;
    CalibUtils::vec9 _A1tb1, _A2tb2, _U9;
    Float::MEmath::Mat<9, 3> _L93;
    
    bool _isOutlierSolutionBased; //< Will be true if frame is outlier based on consistency with current solution
    bool _isOutlierEmConsistency; //< Will be true if frame is outlier based on EM invariants  
    double _gammaI, _gammaII;
    OnlineCalibration::Cam2CamInternalStateInfo _stateInfo;
};
} // namespace EgomotionCalibration