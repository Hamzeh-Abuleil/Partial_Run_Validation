/**
 * \file  c2wOutputIF.h
 * \brief Output Interface to itrk/stdout
 *
 * \author Uri London
 * \date Jul 11, 2019
 */

#ifndef __OC_CAM2WORLD_OUTPUT_IF_H_
#define __OC_CAM2WORLD_OUTPUT_IF_H_

#include "technology/mobilib/float/common/MEmath/mat.h"
#include "technology/calibration/onlineCalibration/Calibrators/StateInfo.h"
#include "technology/calibration/onlineCalibration/Calibrators/ExtrinsicCalibration.h"
#include "basicTypes/containers/fastCyclicVector.h"


#ifdef MEwin
#define OC_C2W_PRINT(m, c, f, ...) do { Cam2WorldOutputIF::instance().tostdout(m, c, f, __VA_ARGS__);} while (0)
// todo: macro for calling general function from outputIF
    const MEtl::string s_State[] = {"GOOD", "UNVALIDATED", "SUSPECTED", "BAD", "NUM_OF_STATES"};
#else
#define OC_C2W_PRINT(m, c, f, ...)
#endif

namespace itrkWriter {
  class ItrkWriterHandle;
}

namespace OnlineCalibration {
  struct OnlineCalibrationModelIF;

  namespace Cam2World {
    class Cam2WorldData;
    class Cam2WorldSignal;
    class PlaneSync;
    struct Plane;

    enum Color {
      e_UNDERLINE    = 1,
      e_HIGHLIGHT    = 7,
      e_GREY         = 30,
      e_RED          = 31,
      e_GREEN        = 32,
      e_YELLOW       = 33,
      e_BLUE         = 34,
      e_PURPLE       = 35,
      e_CYAN         = 36,
      e_WHITE        = 37,
      e_HL_GREY      = 40,
      e_HL_RED       = 41,
      e_HL_GREEN     = 42,
      e_HL_YELLOW    = 43,
      e_HL_BLUE      = 44,
      e_HL_PURPLE    = 45,
      e_HL_CYAN      = 46,
      e_HL_WHITE     = 47,
      e_BO_GREY      = 90,
      e_BO_RED       = 91,
      e_BO_GREEN     = 92,
      e_BO_YELLOW    = 93,
      e_BO_BLUE      = 94,
      e_BO_PURPLE    = 95,
      e_BO_CYAN      = 96,
      e_BO_WHITE     = 97,
      e_HL_BO_GREY   = 100,
      e_HL_BO_RED    = 101,
      e_HL_BO_GREEN  = 102,
      e_HL_BO_YELLOW = 103,
      e_HL_BO_BLUE   = 104,
      e_HL_BO_PURPLE = 105,
      e_HL_BO_CYAN   = 106,
      e_HL_BO_WHITE  = 107,
      e_COLOR_NUM    = 34
    };

    enum Stage {
      e_INPUT        = 1,
      e_VALID        = 2,
      e_CONF         = 4,
      e_RESULT       = 8,
      e_STATE        = 16,
      e_HIST         = 32,
      e_HIST_VERBOSE = 64,
      e_HMM          = 128,
      e_SPC          = 256,
      e_RESERVED2    = 512,
      e_RESERVED3    = 1024 // showAll: -sOCC2W-dbgPrintMask=2047
    };

    class Cam2WorldOutputIF {
      public:
      static Cam2WorldOutputIF& instance();
      ~Cam2WorldOutputIF() {}

      void toItrkHeaders();
      void toItrkInputData(const Cam2WorldData *d) const;
      void toItrkFrameValidation(const Cam2WorldData *d) const;
      void toItrkSignal(const Cam2WorldSignal *s) const;
      void toItrkPlane(const Cam2WorldData *d) const;
      void toItrkPlaneBuffer(const MEtypes::FastCyclicVector<Plane> *p) const;
      void toItrkResult(const Cam2WorldData *d) const;
      void toItrkOutput(State state, int degradeCause, unsigned int highConfTh,
                        unsigned int stateTimeCounter, unsigned int confLevelTimeCounter,
                        unsigned int lowConfTh,
                        unsigned int highConfParams[4], ConfLevel confLevel, State innerState,
                        const ExtrinsicCalibration &ec, const Cam2WorldStateInfo &si) const;
      void toItrkSPC(const Cam2WorldData *d) const;
      void toItrkSPC(SpcData spc) const;
      void toItrkModelIF(const OnlineCalibrationModelIF *m) const;

      void tostdout(int mask, int clr, std::string format, ...);
      // todo: add option for variadic stuff
      void tostdoutRT(int mask, int clr, std::string header,
                      Float::MEmath::Mat<3,3,float> R,
                      Float::MEmath::Vec<3,float> t);
      void tostdoutMatf(int mask, int clr, std::string header,
                        CalibUtils::mat33f M);

      private:
      Cam2WorldOutputIF();

      enum ItrkType {e_INPUT_DATA,
                     e_INPUT_DATA_CONF,
                     e_FRAME_VALIDATION,
                     e_SIGNAL,
                     e_HIST,
                     e_PLANE,
                     e_PLANE_BUFFER,
                     e_RESULT,
                     e_OUTPUT,
                     e_OUTPUT_CONF,
                     e_SPC,
                     e_MODEL_IF,
                     e_ITRK_NUM_TYPES};

      itrkWriter::ItrkWriterHandle *_itrkHandle[e_ITRK_NUM_TYPES];
      CameraInfo::CameraInstance _primaryCam;
      unsigned int _dbgPrintMask; // a mask to control which debug printout lines are printed (enum Stage)
    };

  } // namespace Cam2World
} // namespace OnlineCalibration

#endif // __OC_CAM2WORLD_OUTPUT_IF_H_
