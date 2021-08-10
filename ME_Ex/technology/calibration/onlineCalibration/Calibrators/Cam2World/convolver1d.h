/**
 * \file convolver1d.h
 * \brief ConvolverOneDim header - Class which smoothes a one dimensional set of values
 *
 * \author Uri London
 * \date Jul 22, 2019
 */


#ifndef __OC_CAM2WORLD_CONVOLVER1D_H
#define __OC_CAM2WORLD_CONVOLVER1D_H

#include "technology/calibration/onlineCalibration/Calibrators/Cam2World/c2wConsts.h"
#include "basicTypes/containers/fastCyclicVector.h"

namespace OnlineCalibration {
  namespace Cam2World {

    class ConvolverOneDim {
      public:
        enum KernelType {
          e_KERNEL_EXP,
          e_KERNEL_POWER,
          e_KERNEL_CHI2,
          e_KERNEL_LONGNORM,
          e_KERNEL_UNIFORM_PAST,
          e_KERNEL_UNIFORM,
          e_KERNEL_GAUSS,
          e_KERNEL_NUM,
        };

        ConvolverOneDim();
        ~ConvolverOneDim() {}

        void reset();
        void setKernel(const KernelType kernelType);
        float update(float x=0.f);
        void updateWidth(float width);
        float getWidth(){ return _width;}

        float result() const { return _result;}

      private:
        float kernelExp(unsigned int i);

        float _result;
        float _width;
        float _initWidth;
        float _kernel[CONVOLVE_WIN_SIZE];
        KernelType _kernelType;

        MEtypes::FastCyclicVector<float> _valueBuff; // buffer to holds values
    };

  } // Cam2World
} // OnlineCalibration

#endif // __OC_CAM2WORLD_CONVOLVER1D_H
