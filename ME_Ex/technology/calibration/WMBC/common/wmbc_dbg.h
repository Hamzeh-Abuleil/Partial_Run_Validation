#ifndef _WMBC_DBG_H_
#define _WMBC_DBG_H_

// ------------ DEBUG PRINTS
#define WMBC_DEBUG_PRINT
#undef WMBC_DEBUG_PRINT

struct WmbcDebug{
  enum Color {e_BLACK, e_RED, e_GREEN, e_BROWN, e_BLUE, e_PURPLE, e_CYAN, e_WHITE, e_COLOR_NUM};
  enum Stage {
    e_INPUT                = 1,
    e_VALIDATE_FRAME       = 2,
    e_CONVERGENCE          = 4,
    e_CONFIDENCE           = 8,
    e_RESULTS              = 16,
    e_STATUS               = 32,
    e_FOE_RESET            = 64,
    e_SHOW                 = 128,
    e_MODEL                = 256,
    e_HISTOGRAM            = 512,
    e_HISTOGRAM_VERBOSE    = 1024,
    e_TARGETS              = 2048,
    e_TARGETS_BASE         = 4096,
    e_TARGETS_BASE_VERBOSE = 8192,
    e_AUTOFIX              = 16384,
    e_RESERVED1            = 32768
  };
};
typedef WmbcDebug::Color WmbcDbgClr;
typedef WmbcDebug::Stage WmbcDbgStg;

#define WMBC_CASES4(x, y0, s0, y1, s1, y2, s2, s3) ((x)==(y0) ? s0 : ((x)==(y1) ? s1 : ((x)==(y2) ? s2 : (s3))))
#define WMBC_CASE_3(x, y0, y1, y2) ((x)==(y0) ? #y0 : ((x)==(y1) ? #y1 : #y2))
#define WMBC_CASE_4(x, y0, y1, y2, y3) ((x)==(y0) ? #y0 : ((x)==(y1) ? #y1 : ((x)==(y2) ? #y2 : (#y3))))
#define WMBC_CASE12(x, y0, y1, y2, y3, y4, y5, y6, y7, y8, y9, y10, y11) ((x)==(y0) ? #y0 : ((x)==(y1) ? #y1 : ((x)==(y2) ? #y2 : ((x)==(y3) ? #y3 : ((x)==(y4) ? #y4 : ((x)==(y5) ? #y5 : ((x)==(y6) ? #y6 : ((x)==(y7) ? #y7 : ((x)==y8 ? #y8 : ((x)==(y9) ? #y9 : ((x)==(y10) ? #y10 : #y11)))))))))))
#define WMBC_DOF_ID_STR(x) WMBC_CASE_4(x, e_YAW, e_HORIZON, e_ROLL, e_CAM_HEIGHT)
#define WMBC_STATUS_STR(x) WMBC_CASE12(x, e_CALIB_UNDEFINED, e_CALIB_OK, e_CALIB_CAL, e_CALIB_PAUSED, e_CALIB_RUN_ERROR, e_CALIB_TIMEOUT, e_CALIB_TIMEOUT_TIME, e_CALIB_TIMEOUT_DISTANCE, e_CALIB_ERROR_OUT_OF_RANGE, e_CALIB_FAILSAFE_YAW, e_CALIB_FAILSAFE_HORIZON, e_CALIB_FAILSAFE_ROLL)
#define WMBC_CORE_STATUS_STR(x) WMBC_CASE_3(x, e_CORE_INIT, e_CORE_SUCCESS, e_CORE_ERROR)
#define WMBC_ISLESS(x, y) ((x)<(y) ? "<" : ">=")
#define WMBC_ISGREAT(x, y) ((x)>(y) ? ">" : "<=")

//#define WMBC_NARGS(...)  (sizeof((int[]){__VA_ARGS__})/sizeof(int))

#define _WMBC_ON_STR(x, m) std::string(((x)&(m))==(m) ? #m : "")
#define _WMBC_ON_STR2(x, m1, m2) (_WMBC_ON_STR(x,m1)+" "+_WMBC_ON_STR(x,m2))
#define _WMBC_ON_STR3(x, m1, m2, m3) (_WMBC_ON_STR2(x,m1,m2)+" "+_WMBC_ON_STR(x,m3))
#define _WMBC_ON_STR4(x, m1, m2, m3, m4) (_WMBC_ON_STR3(x,m1,m2,m3)+" "+_WMBC_ON_STR(x,m4))
#define _WMBC_OVERRIDE5(_1, _2, _3, _4, _5, NAME, ...) NAME
#define WMBC_ON_STR(...) _WMBC_OVERRIDE5(__VA_ARGS__, _WMBC_ON_STR4, _WMBC_ON_STR3, _WMBC_ON_STR2, _WMBC_ON_STR)(__VA_ARGS__).c_str()

// #if defined (MEwin) && defined (WMBC_DEBUG_PRINT)
// #include "technology/brain2/prepSys/prepSys_API.h"
// #define DEBUG_LOG_CLR(msg, c) if ((c >=0) && (c < 8)) {std::cout << "\033[3" << c << "m" << msg << "\033[0m\n";}
// #define DEBUG_LOG_CLR_NOLINEBREAK(msg, c) if ((c >=0) && (c < 8)) {std::cout << "\033[3" << c << "m" << msg << "\033[0m ";}
// #define DEBUG_LOG_CLR_FRAME(msg, c) if ((c >=0) && (c < 8)) {std::cout << "\033[3" << c << "m<" << *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD) << "> " << msg << "\033[0m\n";}
// #define DEBUG_LOG_CLR_FRAME_NOLINEBREAK(msg, c) if ((c >=0) && (c < 8)) {std::cout << "\033[3" << c << "m<" << *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD) << "> " << msg << "\033[0m ";}
// #define WMBC_PRINT_BW(msg, ...) do { fprintf(stderr, msg, __VA_ARGS__); } while (0)
// #define WMBC_PRINT(c, msg, ...) do { std::stringstream ss; ss << "\033[3" << c << "m" << msg << "\033[0m\n"; fprintf(stderr, ss.str().c_str(), __VA_ARGS__); } while (0)
// #define WMBC_AF_PRINT(c, msg, ...) if (!_quickMode) { WMBC_PRINT(c, msg, __VA_ARGS__);}
// 
// inline MEtl::string dec2binStr(int d, int repSize) {
//   MEtl::string b = "0b";
//   for (int i = repSize-1; i >= 0; --i) {
//     std::stringstream ss;
//     ss << ((d>>i)&1);
//     b += ss.str().c_str();
//     // b += std::to_string((d>>i)&1); // C++11
//   }
// 
//   return b;
// }
// #else
// #define DEBUG_LOG_CLR(msg, c)
// #define DEBUG_LOG_CLR_NOLINEBREAK(msg, c)
// #define DEBUG_LOG_CLR_FRAME(msg, c)
// #define DEBUG_LOG_CLR_FRAME_NOLINEBREAK(msg, c)
// #define WMBC_PRINT_BW(msg, ...)
// #define WMBC_PRINT(c, msg, ...)
// //#define WMBC_PRINT(c, ...)
// #define WMBC_AF_PRINT(c, msg, ...)
// #endif

#ifdef MEwin
#include "technology/brain2/prepSys/prepSys_API.h"
//#define WMBC_PRINT(c, x, ...) do { fprintf(stderr, getDebugStr(c, x), __VA_ARGS__); } while (0)
// #define WMBC_PRINT(c, x, ...) do { std::stringstream ss; setDebugStr(c, x, ss); fprintf(stderr, ss.str().c_str(), __VA_ARGS__); } while (0)
#define WMBC_PRINT(m, c, x, ...) if (((_debugPrintMask)&(m))==(m)) { std::stringstream ss; setDebugStr(c, x, ss); fprintf(stderr, ss.str().c_str(), __VA_ARGS__); }
#define WMBC_CALL(m, x) if (((_debugPrintMask)&(m))==(m)) {x;}
#define DEBUG_LOG_CLR(msg, c)
#define DEBUG_LOG_CLR_NOLINEBREAK(msg, c)
#define DEBUG_LOG_CLR_FRAME(msg, c)
#define DEBUG_LOG_CLR_FRAME_NOLINEBREAK(msg, c)

inline MEtl::string dec2binStr(int d, int repSize) {
  MEtl::string b = "0b";
  for (int i = repSize-1; i >= 0; --i) {
    std::stringstream ss;
    ss << ((d>>i)&1);
    b += ss.str().c_str();
    // b += std::to_string((d>>i)&1); // C++11
  }

  return b;
}

//inline const char* getDebugStr(int clr, std::string x) {
inline const char* getDebugStr(int clr, const char* x) {
  int grabIdx = *PrepSys_API::getGrabIndex(PrepSys::exp_mask::T0, CameraInfo::e_FORWARD);
  std::stringstream ss;
  ss << "<" << grabIdx << "> ";
  if (clr >=0 && clr < 8) {
    ss << "\033[3" << clr << "m" << x << "\033[0m";
  } else {
    ss << x;
  }
  ss << "\n";
  //std::cout << ss.str().c_str();
  fprintf(stderr,"%s", ss.str().c_str());
  return ss.str().c_str();
}

inline void setDebugStr(int clr, const char* x, std::stringstream& ss) {
  if (clr >=0 && clr < 8) {
    ss << "\033[3" << clr << "m" << x << "\033[0m";
  } else {
    ss << x;
  }
  ss << "\n";
}

#else
//#define WMBC_PRINT(m, c, x, ...)
#define WMBC_PRINT(m, c, x, ...)
#define WMBC_CALL(m, x)
#define DEBUG_LOG_CLR(msg, c)
#define DEBUG_LOG_CLR_NOLINEBREAK(msg, c)
#define DEBUG_LOG_CLR_FRAME(msg, c)
#define DEBUG_LOG_CLR_FRAME_NOLINEBREAK(msg, c)
#endif

// ------------ PROFILING FUNCTIONS! (copied from avrozen's technology/road2/common/gsf_prof.h
#ifdef EYEQ_HW_IMPL
#define WMBC_GSF_PROF   
#undef WMBC_GSF_PROF   //should be NOT remarked in committed code
#define WMBC_MAN_PROF   
#undef WMBC_MAN_PROF   //should be NOT remarked in committed code
#define WMBC_MAN_MEMCHECK
#undef WMBC_MAN_MEMCHECK   //should be NOT remarked in committed code
#endif

#define WMBC_GSF_LIKE_PC_PRINTS 
#undef WMBC_GSF_LIKE_PC_PRINTS  //should be NOT remarked in committed code

#ifdef WMBC_GSF_PROF
#include "utilities/htcore-3/include/htc_time.h"
#include "functionality/gsf/gsf_logf.h"
#include "technology/brain2/brain2_API.h"
#define WMBC_GSF_PROF_START(name) GSF_PROF_START(name)
#define WMBC_GSF_PROF_FINISH(name) GSF_PROF_FINISH(name)
#define WMBC_GSF_PROF_SCOPE(name) GSF_PROF_SCOPE(name)
#define WMBC_GSF_PROF_START_ID(name, id) logf(&g_gsf_logf_prof_buffer, "start_id  " name " %d %u\n", id, (unsigned)GSF_NOW)
#define WMBC_GSF_PROF_FINISH_ID(name, id) logf(&g_gsf_logf_prof_buffer, "finish_id " name " %d %u\n", id, (unsigned)GSF_NOW)
#define WMBC_GSF_PROF_VAR(name, id, fmt, value) logf(&g_gsf_logf_prof_buffer, "var " name " id %d " fmt "\n", id, value)
#define WMBC_GSF_PROF_DO(DO) do {DO;}while(0);
#define WMBC_GSF_DECLARE_VAR(DECLARATION) DECLARATION;//dont add ; at usage
#elif defined(WMBC_GSF_LIKE_PC_PRINTS)
#define WMBC_GSF_PROF_START(name)
#define WMBC_GSF_PROF_FINISH(name)
#define WMBC_GSF_PROF_SCOPE(name)
#define WMBC_GSF_PROF_START_ID(name, id)
#define WMBC_GSF_PROF_FINISH_ID(name, id)
#define WMBC_GSF_PROF_VAR(name, id, fmt, value) printf("var " name " id %d " fmt "\n", id, value);
#define WMBC_GSF_PROF_DO(DO) do {DO;}while(0);
#define WMBC_GSF_DECLARE_VAR(DECLARATION) DECLARATION; //dont add ; at usage
#else  // no printing nonsense at all
#define WMBC_GSF_PROF_START(name)
#define WMBC_GSF_PROF_FINISH(name)
#define WMBC_GSF_PROF_SCOPE(name)
#define WMBC_GSF_PROF_START_ID(name, id)
#define WMBC_GSF_PROF_FINISH_ID(name, id)
#define WMBC_GSF_PROF_VAR(name, id, fmt, value)
#define WMBC_GSF_PROF_DO(DO);
#define WMBC_GSF_DECLARE_VAR(DECLARATION) //dont add ; at usage
#endif

#ifdef WMBC_MAN_PROF
#include "technology/mobilib/fix/common/MEXmisc/cycleTimer.h"
#include "functionality/gsf/gsf_logf.h"
#define WMBC_DTIMER(t) static MEmisc::CycleTimer t
#define WMBC_STIMER(t) do { (t).reset();(t).start(); } while(0)
//#define WMBC_ETIMER(t,msg) do { (t).stop(); gsf_logf(msg "0x%08X%08X", (unsigned int)((t).total()>>32), (unsigned int)((t).total()&0xFFFFFFFFULL));} while(0)
#define WMBC_ETIMER(t,msg) do { (t).stop(); gsf_logf(msg " total: %u, cycles: %f, cyclesPerSec: %f\n", (unsigned int)((t).total()), (float)(t).cycles(), (float)(t).cyclesPerSecond());} while(0)
#else
#define WMBC_DTIMER(t)
#define WMBC_STIMER(t)
#define WMBC_ETIMER(t,msg)
#endif

#ifdef WMBC_MAN_MEMCHECK
#include "functionality/gsf/gsf_logf.h"
#include "technology/SSPA/lib/ARM/SSPA_HeapSection.h"
extern SSPA::HeapSection g_heapSection;
// #define WMBC_OBJECT_SIZE(msg) gsf_logf(msg " size of %s: %d", typeid(*this).name(), sizeof(*this))
#define WMBC_THIS_OBJECT_SIZE(msg) gsf_logf(msg " my size: %d\n", (int)(sizeof(*this)))
#define WMBC_OBJECT_SIZE(msg, obj) gsf_logf(msg " my size: %d\n", (int)(sizeof(obj)))
#define WMBC_HEAP_STATUS(msg) gsf_logf(msg " heap, consumed: %d out of : %d\n", (int)(g_heapSection.curr - g_heapSection.begin), (int)(g_heapSection.end - g_heapSection.begin))
#else
#define WMBC_THIS_OBJECT_SIZE(msg)
#define WMBC_OBJECT_SIZE(msg, obj)
#define WMBC_HEAP_STATUS(msg)
#endif

#endif // _WMBC_DBG_H_

