#include "utilities/cameraInformation/cameraInformation_API.h"
#include "technology/memory/lib/common/lock.h"
#include "technology/stdTrixes/new/ME_memory.h" // default_malloc

namespace CameraInfo{
  CameraInstance getSourceInstance(CameraInstance inst) {return e_FORWARD;}
  CameraInstance itrkNoCameraData(){return e_NO_CAM_ASSOCIASION;}
  bool exists(CameraInfo::CameraInstance) { return false; }
}

namespace VehicleInfo {
    bool movablePartsUnlocked(int expIdx=0) { return false; }
    bool movablePartsUnlockedValid(int expIdx=0) { return false; }
}

MEIO::MultipleReadersOneWriter::MultipleReadersOneWriter(){}
MEIO::WriteGuard::WriteGuard(MEIO::MultipleReadersOneWriter& lock, MEIO::LockLog*): _lock(lock){}
MEIO::WriteGuard::~WriteGuard(){}
MEIO::ReadGuard::ReadGuard(MEIO::MultipleReadersOneWriter& lock, MEIO::LockLog*): _lock(lock){}
MEIO::ReadGuard::~ReadGuard(){}

void* default_malloc(size_t bytes, const char* debug)
{ 
   return malloc(bytes);
}
void default_free(void *p) 
{
   free(p);
}
void userException(const char* reason, const char* file, int line, const char* func)
{
  throw "Exception";
}

namespace Memory
{
  void initlock(mutex_t* mtx) {}
  void lock(mutex_t* obj){}
  void unlock(mutex_t* obj){}
}