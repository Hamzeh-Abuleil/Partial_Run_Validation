#include "technology/calibration/onlineCalibration/CalibUtils/calibTypes.h"

namespace CalibUtils
{

    Float::MEmath::Mat<3, 3, double> matf2d(Float::MEmath::Mat<3, 3, float> matf) {
      Float::MEmath::Mat<3, 3, double> matd;
      for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
          matd(i, j) = (double)matf(i, j);
        }
      }
      return matd;
    }

    Float::MEmath::Mat<3, 3, float> matd2f(Float::MEmath::Mat<3, 3, double> matd) {
      Float::MEmath::Mat<3, 3, float> matf;
      for (int i=0; i<3; ++i) {
        for (int j=0; j<3; ++j) {
          matf(i, j) = (float)matd(i, j);
        }
      }
      return matf;
    }

    Float::MEmath::Mat<4, 4, float> matd2f(Float::MEmath::Mat<4, 4, double> matd) {
      Float::MEmath::Mat<4, 4, float> matf;
      for (int i=0; i<4; ++i) {
        for (int j=0; j<4; ++j) {
          matf(i, j) = (float)matd(i, j);
        }
      }
      return matf;
    }

    Float::MEmath::Vec<3, float> vecd2f(Float::MEmath::Vec<3, double> vecd) {
      Float::MEmath::Vec<3, float> vecf;
      for (int i=0; i<3; ++i) {
        vecf[i]= (float)vecd[i];
      }
      return vecf;
    }
}
