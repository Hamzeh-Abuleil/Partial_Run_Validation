#pragma once
#include <stdint.h>
#include <string>
#include <unordered_set>

#include "functionality/partialRun/wrappers/IFTypeList.h"
#include "technology/mobilib/fix/common/MEXimage/sync.h"
#include "utilities/pext/pext.h"
#include "functionality/partialRun/includes/b2bDefs.h"


namespace PartialRun {
#define FILE_SIZE 0x350000 // 3.5M TODO: reduce this size it need to be 2M increcsed because of issue with VD3DCollectionIF

enum class IFCategoryShift : uint64_t
{
	Slow = 0,
	Fast = 1,
	PED_IF = 2,
	VD3D_VEHICLES_POST_DETECTION_PR = 3,
	VD3D_IF_SA_PR = 4,
	VCL_LIGHTINGCONDITION = 5,
	VCL_CLIQUESMEASUREMENTS = 6,
	VCL_CLIQUES_UPDATE = 7,
	VCL_POST_CROSSING_CIPV_DETECTION = 8,
	VCL_CLIQUESMEASUREMENTS_CLIQUES = 9,
	WMBC_PR = 10,
	MEMEM_INIT_FRAME_PR = 11,
	MEMEM_SPEED_FACTOR_PR = 12,
	MEMEM_CURR_VEH_RT_PR = 13,
	HRSPOTS_SHARED_DATA_PR = 14,
	HRSPOTS_SHARED_RUNNING_MODE_PR = 15,
	HRSPOTS_SHARED_SCENE_DETECTION_PR = 16,
	FAIL_SAFES_END_FS = 17,
	DS_SLI_IF_END_FRAME = 18,
	PED_FCW_IF = 19,
	EMODEL = 20,
	VD_PED_RELATION = 21,
	ROADT0_STORE_FAST_FOR_SLOW = 22,
	VRM_SLOW = 23,
	VCL_COLLECTIONS = 24,
	SEMANTIC_LANES_DESC = 25,
	RFD_FREE_SPACE = 26,
	REFS_FS = 27,
	NNFS_FS = 28,
	HRSPOTS_IF_FL = 29,
	ONLINE_CALIBRATION_PR = 30,
	SCMIF_MODEL = 31,
	REIF_MODEL = 32,
	SENSING_DATA = 33,
	PED_CLIQUE_IF = 34,
	DSIF_END_FRAME = 35,
	PEDAL_CONFUSION_SLOW = 36,
    HAZARD_FILL_IF = 37,
    REGION_CODE_DECIDER = 38,
	ROAD_FILL_IF = 39,
	FCW_FILL_IF = 40,
	IFCategory_COUNT //this value indicate the count of the enumerators in IFCategory
};

enum class IFCategory : uint64_t
{
	Slow = ONE_64 << (uint64_t)IFCategoryShift::Slow,
	Fast = ONE_64 << (uint64_t)IFCategoryShift::Fast,
	PED_IF = ONE_64 << (uint64_t)IFCategoryShift::PED_IF,
	VD3D_VEHICLES_POST_DETECTION_PR = ONE_64 << (uint64_t)IFCategoryShift::VD3D_VEHICLES_POST_DETECTION_PR,
	VD3D_IF_SA_PR = ONE_64 << (uint64_t)IFCategoryShift::VD3D_IF_SA_PR,
	VCL_LIGHTINGCONDITION = ONE_64 << (uint64_t)IFCategoryShift::VCL_LIGHTINGCONDITION,
	VCL_CLIQUESMEASUREMENTS = ONE_64 << (uint64_t)IFCategoryShift::VCL_CLIQUESMEASUREMENTS,
	VCL_CLIQUES_UPDATE = ONE_64 << (uint64_t)IFCategoryShift::VCL_CLIQUES_UPDATE,
	VCL_POST_CROSSING_CIPV_DETECTION = ONE_64 << (uint64_t)IFCategoryShift::VCL_POST_CROSSING_CIPV_DETECTION,
	VCL_CLIQUESMEASUREMENTS_CLIQUES = ONE_64 << (uint64_t)IFCategoryShift::VCL_CLIQUESMEASUREMENTS_CLIQUES,
	WMBC_PR = ONE_64 << (uint64_t)IFCategoryShift::WMBC_PR,
	MEMEM_INIT_FRAME_PR = ONE_64 << (uint64_t)IFCategoryShift::MEMEM_INIT_FRAME_PR,
	MEMEM_SPEED_FACTOR_PR = ONE_64 << (uint64_t)IFCategoryShift::MEMEM_SPEED_FACTOR_PR,
	MEMEM_CURR_VEH_RT_PR = ONE_64 << (uint64_t)IFCategoryShift::MEMEM_CURR_VEH_RT_PR,
	HRSPOTS_SHARED_DATA_PR = ONE_64 << (uint64_t)IFCategoryShift::HRSPOTS_SHARED_DATA_PR,
	HRSPOTS_SHARED_RUNNING_MODE_PR = ONE_64 << (uint64_t)IFCategoryShift::HRSPOTS_SHARED_RUNNING_MODE_PR,
	HRSPOTS_SHARED_SCENE_DETECTION_PR = ONE_64 << (uint64_t)IFCategoryShift::HRSPOTS_SHARED_SCENE_DETECTION_PR,
	FAIL_SAFES_END_FS = ONE_64 << (uint64_t)IFCategoryShift::FAIL_SAFES_END_FS,
	DS_SLI_IF_END_FRAME = ONE_64 << (uint64_t)IFCategoryShift::DS_SLI_IF_END_FRAME,
	PED_FCW_IF = ONE_64 << (uint64_t)IFCategoryShift::PED_FCW_IF,
	EMODEL = ONE_64 << (uint64_t)IFCategoryShift::EMODEL,
	VD_PED_RELATION = ONE_64 << (uint64_t)IFCategoryShift::VD_PED_RELATION,
	ROADT0_STORE_FAST_FOR_SLOW = ONE_64 <<(uint64_t)IFCategoryShift::ROADT0_STORE_FAST_FOR_SLOW,
	VRM_SLOW = ONE_64<<(uint64_t)IFCategoryShift::VRM_SLOW,
	VCL_COLLECTIONS = ONE_64 << (uint64_t)IFCategoryShift::VCL_COLLECTIONS,
	SEMANTIC_LANES_DESC = ONE_64 <<(uint64_t)IFCategoryShift::SEMANTIC_LANES_DESC,
	RFD_FREE_SPACE = ONE_64 <<(uint64_t)IFCategoryShift::RFD_FREE_SPACE,
	REFS_FS = ONE_64 <<(uint64_t)IFCategoryShift::REFS_FS,
	NNFS_FS = ONE_64 <<(uint64_t)IFCategoryShift::NNFS_FS,
	HRSPOTS_IF_FL = ONE_64 <<(uint64_t)IFCategoryShift::HRSPOTS_IF_FL,
	ONLINE_CALIBRATION_PR = ONE_64 <<(uint64_t)IFCategoryShift::ONLINE_CALIBRATION_PR,	
	SCMIF_MODEL = ONE_64 <<(uint64_t)IFCategoryShift::SCMIF_MODEL,
	REIF_MODEL = ONE_64 <<(uint64_t)IFCategoryShift::REIF_MODEL,
	SENSING_DATA = ONE_64 <<(uint64_t)IFCategoryShift::SENSING_DATA,
	PED_CLIQUE_IF = ONE_64 <<(uint64_t)IFCategoryShift::PED_CLIQUE_IF,
	DSIF_END_FRAME = ONE_64 <<(uint64_t)IFCategoryShift::DSIF_END_FRAME,
	PEDAL_CONFUSION_SLOW = ONE_64 <<(uint64_t)IFCategoryShift::PEDAL_CONFUSION_SLOW,
	HAZARD_FILL_IF = ONE_64 <<(uint64_t)IFCategoryShift::HAZARD_FILL_IF,
    REGION_CODE_DECIDER = ONE_64 <<(uint64_t)IFCategoryShift::REGION_CODE_DECIDER,
	ROAD_FILL_IF = ONE_64 <<(uint64_t)IFCategoryShift::ROAD_FILL_IF,
	FCW_FILL_IF = ONE_64 <<(uint64_t)IFCategoryShift::FCW_FILL_IF,

};

enum class IFExposure: uint32_t
{
	T0 = 4,
	C0 = 5,
	T1 = 6,
	C1 = 7
};

//Below array for needed for PextStore, it has to the same order with enum IFCategoryShift
#if !defined(EYEQ_HW_IMPL) && !defined(OS_FAMILY_WINDOWS)
extern const char * IFCategoryShiftStr[(int)PartialRun::IFCategoryShift::IFCategory_COUNT] ;
#endif

template<typename E>
auto as_underlying(E e) -> std::underlying_type_t<E>
{
	return static_cast<std::underlying_type_t<E>>(e);
}

inline IFCategory operator | (IFCategory lhs, IFCategory rhs)
{
	return static_cast<IFCategory>(as_underlying(lhs) | as_underlying(rhs));
}

inline IFCategory operator & (IFCategory lhs, IFCategory rhs)
{
	return static_cast<IFCategory>(as_underlying(lhs) & as_underlying(rhs));
}

class BaseIFWrapper
{
public:
	IFType _type;
	PRTechType _techType;
	const char* _techTypeName;
	uint32_t _index;
  brainId _brainId;
	const char* _indexName;
	IFCategory _category;
	void* _data;
	uint32_t _size;
	const char *_type_name;
	const char *_name;
	const char *_hash;
	const char *_path;
	void *_user;
	std::unordered_set<std::string> _whitelist;

	BaseIFWrapper(IFType type,
		PRTechType techType,
		const char* techTypeName,
		uint32_t index,
    brainId brainId,
		IFCategory category,
		void *data,
		uint32_t size,
		const char *type_name,
		const char *name,
		const char *hash,
		const char *path);

	virtual ~BaseIFWrapper()
	{
		//  std::cout << "wrapper was distoyed: " << _name << std::endl;
	}

	virtual void update(){ }

	virtual BaseIFWrapper* clone() { assert(false); return nullptr;}

	virtual void PostDeserialize() { update(); }

#if !defined(EYEQ_HW_IMPL) && !defined(OS_FAMILY_WINDOWS) && defined(PEXT_PR)
	virtual void dump2pext(std::string pextName, uint32_t expId)=0;

	virtual void readFromPext(std::string pextName, uint32_t expId, uint32_t grabIndex)=0;
#endif

	bool is_tech_allowed() const;
	static void updatePartialRunStore(const BaseIFWrapper* wrapper, 
																		IFCategory category,
																		uint32_t expIndex = 0);
	static void updatePartialRunLoad(const BaseIFWrapper* wrapper, 
																		IFCategory category,
																		uint32_t expIndex = 0);
#if !defined(EYEQ_HW_IMPL) && !defined(OS_FAMILY_WINDOWS) && defined(PEXT_PR)
template<typename T, size_t size> 
		void dumpArray2Pext(std::string pextName, const std::string arrayName, const T (&array)[size], uint32_t expId)
		{
			pextName = pextName + "." + arrayName;
			for(size_t i=0; i < size; ++i)
			{
				Pext_SetData(pextName, "index", &_index);
				Pext_SetData(pextName, "expId", &expId);
				Pext_SetGeneral(pextName, arrayName, &array[i]);
				Pext_FlushTuple(pextName);
			}   
    }	

		template<typename T, size_t size> 
		void readArrayFromPext(const std::string arrayName, const T (&array)[size], std::string pextName, uint32_t expId, uint32_t grabIndex, uint32_t wrapperIndex)
		{
      pextName = pextName + "." + arrayName;
      PextIO::PextRecord record;
      uint32_t rowExpId, index;
			
      record = Pext_GetRecord(pextName, grabIndex);
      size_t rowsNum = record.rowsNum();
      
      for (size_t currRow = 0; currRow < rowsNum; ++currRow)
      {
        record.get(currRow, "expId", &rowExpId);
				record.get(currRow, "index", &index);

        if (expId == rowExpId && index == wrapperIndex)
        {
					for (size_t i = 0; i < size; ++i)
					{
						record.getGeneral(currRow + i, arrayName, &array[i]);
					}
					break; 				         
        }
      } 
    }	
#endif
	};

template<class T>
class BaseIFWrapperSync : public BaseIFWrapper
{
public:
	Fix::MEimage::Sync<T>* _syncObj;

	BaseIFWrapperSync(IFType type,
		PRTechType techType,
		const char* techTypeName,
		uint32_t index,
    brainId brainId,
		IFCategory category,
		Fix::MEimage::Sync<T>* data,
		uint32_t size,
		const char* type_name,
		const char* name,
		const char* hash,
		const char* path) :
			BaseIFWrapper(
				type, techType, techTypeName, index, brainId, category,
				&data->editable(), size, type_name, name, hash,
				path),
			_syncObj(data)
	{
	}

	void update() override{
		BaseIFWrapper::update();
		_syncObj->update();
	}
};
} // namespace PartialRun
