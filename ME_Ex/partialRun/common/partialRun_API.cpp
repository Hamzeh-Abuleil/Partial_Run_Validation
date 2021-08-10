#include "functionality/partialRun/common/dataRepositoryComposite.h"
#include "functionality/partialRun/common/prProperties.h"
#include "functionality/partialRun/common/partialRun_API_internal.h"
#include "functionality/partialRun/common/partialRunService.h"
#include "functionality/partialRun/brain2Brain_API.h"
#include "functionality/partialRun/common/b2bProperties.h"
#include "technology/brain2/brain2_API.h"

PartialRun::CompositeDataRepository &dataRepoInstance()
{
	static PartialRun::CompositeDataRepository global;
	return global;
}

bool PartialRun_API::isTechDisabledByDummyMode(PartialRun::PRTechType techType)
{
	return dataRepoInstance().isTechDisabledByDummyMode(techType);
}

bool PartialRun_API::isTechDisabledByLightPlayMode(PartialRun::PRTechType techType)
{
	return dataRepoInstance().isTechDisabledByLightPlayMode(techType);
}

bool PartialRun_API::isTechDisabledByPartialRun(PartialRun::PRTechType techType)
{
	return dataRepoInstance().isTechDisabledByPartialRun(techType);
}

bool PartialRun_API::isSepEnabledByThisLevel(PartialRun::LayerLevel level)
{
	return dataRepoInstance().isSepEnabledByThisLevel(level);
}

bool PartialRun_API::isPartialRunDisabled()
{
	return dataRepoInstance().isPartialRunDisabled();
}

void PartialRun_API::registerIF (PartialRun::BaseIFWrapper *IF)
{
	dataRepoInstance().registerIF(IF);
}

const PartialRun::BaseIFWrapper *PartialRun_API::getIfWrapper(PartialRun::IFType type, uint32_t index, PartialRun::brainId brainId)
{
  return dataRepoInstance().getIF(type, index, brainId);
}

PartialRun::brainId PartialRun_API::getWhichBrainIdAmI()
{
  return dataRepoInstance().getWhichBrainIdAmI();
}

const std::vector<PartialRun::brainId>& PartialRun_API::getAvailableConnectedBrains()
{
  return dataRepoInstance().getAvailableConnectedBrains();
}

void PartialRun_API::initProperties()
{
  dataRepoInstance().initProperties(new PartialRun::PropertiesFactory());

  PartialRunService_API* prService = new PartialRunService();
  ServiceLocator_API::ServiceLocator::instance()->advertise(prService);
}

void PartialRun_API::init ()
{
  PartialRun::StoreFactory* storeFactory = new PartialRun::StoreFactory();
  PartialRun::PropertiesFactory* propFactory = new PartialRun::PropertiesFactory();
  storeFactory->init(propFactory);
  dataRepoInstance().init(storeFactory, propFactory);
  dataRepoInstance().initStore();
#if EYEQ_REV>=5
  // Set data Repositories per connected brain following the B2B prop inputs
  PartialRun::BrainToBrainProps* b2bProps = propFactory->createb2bProperties();
  if(b2bProps->getB2BEnable() && b2bProps->getWhoAmI()==1) //rear brain
  {
    // this IF's duplicate on rear and front cam
    Brain2API::setExternalsEIF(Brain2API::getRoadExternalsEIF(),      1, Brain2Brain_API::geteRoadIFVec()[1]);
    Brain2API::setExternalsEIF(Brain2API::getObstaclesExternalsEIF(), 1, Brain2Brain_API::geteObstaclesIFVec()[1]);
    #ifdef HPAD_DELIVERED_TO_BUNDLE
    Brain2API::setExternalsEIF(Brain2API::getSliExternalsEIF(), 1, dataRepoInstance().getEDSIFVec()[1]); // TODO: Yuvalm change to getEDSExternalsEIF()
    Brain2API::setExternalsEIF(Brain2API::getSdmExternalsEIF(), 1, dataRepoInstance().getESDMIFVec()[1]);
    Brain2API::setExternalsEIF(Brain2API::getFailSafesExternalsEIF(), 1, dataRepoInstance().getEFailSafesIFVec()[1]);
    Brain2API::setExternalsEIF(Brain2API::getCrcExternalsEIF(), 1, dataRepoInstance().getECRCIFVec()[1]);
    #endif
    //This IF's runs only on front cam so it get the one and only reference.
    Brain2API::setGeneralObjectsIF(nullptr, dataRepoInstance().getEHazardIFVec()[0]);
    Brain2API::setPedFCWIF(nullptr, dataRepoInstance().getEPedFCWIFec()[0]);
    Brain2API::setSceneUnderstandingIF(nullptr, dataRepoInstance().getESceneUnderstandingIFVec()[0]);
    Brain2API::setFCWIF(nullptr, dataRepoInstance().getEFCW_IFVec()[0]);
    Brain2API::setHRSpotsIF(nullptr, dataRepoInstance().getEHRSpotsIFVec()[0]);
    Brain2API::setSceneManagementIF(nullptr, dataRepoInstance().getEScmIFVec()[0]);
  }
#endif

}

void PartialRun_API::updatePartialRunStore(const PartialRun::BaseIFWrapper* wrapper, PartialRun::IFCategory category, uint32_t expIndex)
{
	PartialRun::BaseIFWrapper::updatePartialRunStore(wrapper, category, expIndex);
}

void PartialRun_API::updatePartialRunLoad(const PartialRun::BaseIFWrapper* wrapper, PartialRun::IFCategory category, uint32_t expIndex)
{
	PartialRun::BaseIFWrapper::updatePartialRunLoad(wrapper, category, expIndex);
}

void PartialRun_API::term()
{
  return dataRepoInstance().term();
}