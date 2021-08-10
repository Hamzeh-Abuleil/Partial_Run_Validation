/*
 * partialRun_API.h
 *
 * Use this file for enableing partial run abilty on your technology,
 * for allowing disable of your tech for each SEP_<tech> in your code call to:
 * RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(PartialRun::PRTechType::<TECH_NAME>)
 *  @ tech name will includ in PRTechType only after adding this teach to: listOfIFs.json4
 * ME.Develop/functionality/partialRun/python/listOfIFs.json
 * than Partial run auto generated enum with your technology prerequesist.
 *
 *  Created on: Jan 27, 2019
 *      Author: eladb
 */

#ifndef PARTIALRUN_API_H_
#define PARTIALRUN_API_H_

#include "functionality/partialRun/wrappers/IFTypeList.h"
#include "functionality/partialRun/common/b2bCommonDefs.h"

#define RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN(techType) {} if (PartialRun_API::isTechDisabledByPartialRun(techType)) return
#define ACTIVE_IF_SEP_ASSOCIATED_WITH_THIS_LEVEL(level) {} if (!PartialRun_API::isSepEnabledByThisLevel(level)) return
// #define RETURN_IF_TECH_DISABLED_BY_PARTIAL_RUN_EX(techType, level) {} if (PartialRun_API::isTechDisabledByPartialRun(techType)||!PartialRun_API::isSepEnabledByThisLevel(level)) return
namespace PartialRun
{
  enum class PRTechType: uint32_t;
  enum class IFCategory : uint64_t;
  enum class IFType: uint32_t;
  class BaseIFWrapper;
}

namespace PartialRun_API
{
  /**
  * getIfWrapper   This function returns IFWrapper instances of the given index in the vector
  *           of this IFtype.
  *           Don't use this function if you don't know your PartialRun::IFType!!!
  *           use getIF(index, brainId) insted.
  * @param  - type: IfType enum of the given wrapper.
  * @param  - index: index of the wanted if from the given IfType vector of this IF.
  * @param  - brainId: id of the brain that we need to get it's IF wrapper, 0 by defult.
  * @return - IF Wrapper.
  */
  const PartialRun::BaseIFWrapper * getIfWrapper(PartialRun::IFType type, uint32_t index, PartialRun::brainId brainId);
  /**
  * getIF     This function returns IFWrapper instances of the given index in the vector
  *           of this IFtype.
  * @param  - index: index of the wanted if from the given IfType vector of this IF.
  * @param  - brainId: id of the brain that we need to get it's IF wrapper, -1 by defult.
  * @return - IF Wrapper or null pointer if no IF in this brainId index from the wanted IF type.
  * @example- const PedCollectionIF_wrapper * Valname = PartialRun_API::getIF<PedCollectionIF_wrapper>(index, brainId);
  */
  template <class T>
  const T     *getIF                      (uint32_t index = 0, PartialRun::brainId brainId=-1)
  {
    PartialRun::IFType type = T::getType();
    return static_cast<const T *>(getIfWrapper(type, index, brainId));
  }

  /**
  * getWhichBrainIdAmI     This function returns brainId of the current brain.
  * @param  - No param.
  * @return - PartialRun::brainId brain Id of this brain.
  */
  PartialRun::brainId                     getWhichBrainIdAmI();

  /**
  * getAvailableConnectedBrains     This function returns vector with all the connected brains in the system.
  * @param  - No param.
  * @return - Reference to std::vector<PartialRun::brainId> empty if no connected brain.
  */
  const std::vector<PartialRun::brainId>&  getAvailableConnectedBrains();

  /**
  * isTechDisabledByPartialRun     This function check if specific Tech disabled by Partial
  *           Run it means that this tech won't run and TECH_IF_DATA will loaded from previous
  *           partial run 'Record'
  * @param  - techType: technology type should be one of the partial run supported technologyes.
  * @NOTE   - supported techs list can found in compilation output:
  *           ME.Develop/out/<EyeQ#sw_###>/functionality/partialRun/wrappers/IFTypeList.h
  */
  bool        isTechDisabledByPartialRun  (PartialRun::PRTechType techType);

  /**
  * isTechInDummyMode     This function check if specific Tech is disable by dummy mode
  *           It means that this tech won't run and dummy data will be loaded to its IFs
  * @param  - techType: technology type should be one of the partial run supported technologyes.
  * @NOTE   - supported techs list can found in compilation output:
  *           ME.Develop/out/<EyeQ#sw_###>/functionality/partialRun/wrappers/IFTypeList.h
  */
  bool        isTechDisabledByDummyMode           (PartialRun::PRTechType techType);

  /*
  * isTechDisabledByLightPlayMode     This function check if specific Tech is disable by lightPlay mode
  *           It means that this tech won't run and dummy data will be loaded to its IFs
  * @param  - techType: technology type should be one of the partial run supported technologyes.
  * @NOTE   - supported techs list can found in compilation output:
  *           ME.Develop/out/<EyeQ#sw_###>/functionality/partialRun/wrappers/IFTypeList.h
  */
  bool        isTechDisabledByLightPlayMode           (PartialRun::PRTechType techType);

  /*
  * isSepEnabledByThisLevel     This function check if specific SEP should or should not run
  *           for the given Level.
  * @param  - level: layer Level the optional Level types is {lowLevel, mediumLevel, highLevel, commonLevel, All}
  * @NOTE   - supported Level list read from compilation output:
  *           ME.Develop/out/<EyeQ#sw_###>/functionality/partialRun/wrappers/IFTypeList.h
  */
  bool        isSepEnabledByThisLevel           (PartialRun::LayerLevel level);

  /**
  * registerIF     This function register IFWrapper to data repository data base.
  *           it called at init from each technology that use partial run abilities for
  *           store or load its IFwrappers.
  * @param  - IF: pointer to BaseIFWrapper that need to be store or load by technology.
  * @return - No return value.
  * @error  - optional Error msg for internal iStore failure
  */
  void        registerIF                  (PartialRun::BaseIFWrapper *IF);

  /**
  * updatePartialRunStore     This function stores data of stucture wrapped by appropriate wrapper with data.
  *           it has to called at every Sep where IF structures data was filled, and where IF has it's own category,
  *           defined in it json file, slow/fast categories has to be supported automatically
  * @param  - wrapper: pointer to BaseIFWrapper that need to be store or load by technology.
  * @param  - category: IF category defined in json file (example: see in ME.Develop/technology/objectSensing/utilities/duplications/vdPedsRelationsIF.h.VDPedsRelationCollectionIF.json),
  * and added to BaseIFWrapper enums.
  * @param  - expIndex: exposure index may be: T0/C0/T1/C1. If 4 exposures were stored, 4 loads has to be called
  * @return - No return value.
  * @error  - optional Error msg for internal iStore failure
  */
  void updatePartialRunStore(const PartialRun::BaseIFWrapper* wrapper, PartialRun::IFCategory category, uint32_t expIndex = 0);

  /**
  * updatePartialRunLoad     This function loads data of stucture wrapped by appropriate wrapper from data repository.
  *           it has to called at every Sep where IF structures data was filled, and where IF has it's own category,
  *           defined in it json file, slow/fast categories has to be supported automatically
  * @param  - wrapper: pointer to BaseIFWrapper that need to be store or load by technology.
  * @param  - category: IF category defined in json file (example: see in ME.Develop/technology/objectSensing/utilities/duplications/vdPedsRelationsIF.h.VDPedsRelationCollectionIF.json),
  * and added to BaseIFWrapper enums.
  * @param  - expIndex: exposure index may be: T0/C0/T1/C1. If 4 exposures were stored, 4 loads has to be called
  * @return - No return value.
  * @error  - optional Error msg for internal iStore failure
  */
  void updatePartialRunLoad(const PartialRun::BaseIFWrapper* wrapper, PartialRun::IFCategory category, uint32_t expIndex = 0);

  /**
  * isPartialRunDisabled     This function check if Partial Run disabled in Application, it means that we are not in Play mode
  * possible we only in record or no partial run at all
  * @param  - No.
  * @NOTE   - supported techs list can found in compilation output:
  *           ME.Develop/out/<EyeQ#sw_###>/functionality/partialRun/wrappers/IFTypeList.h
  */
  bool        isPartialRunDisabled        ();


  /**
  * term     This function terminate the partial run, and will flush any data that was fstored until now.
  * it must be called at the end of the application
  * @param  - No.
  */
  void        term        ();
}
#endif /* PARTIALRUN_API_H_ */
