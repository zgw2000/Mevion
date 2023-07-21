/**********************************************************************************
** THIS SOFTWARE IS THE SOLE PROPERTY OF MEVION MEDICAL SYSTEMS, INC.            **
** ANY USE, REPRODUCTION, OR DISTRIBUTIONS AS A WHOLE OR IN PART WITHOUT THE     **
** WRITTEN PERMISSION OF MEVION MEDICAL SYSTEMS, INC. IS PROHIBITED. THIS NOTICE **
** MAY NOT BE REMOVED WITHOUT WRITTEN APPROVAL FROM MEVION MEDICAL SYSTEMS, INC. **
**                                                                               **
** COPYRIGHT (c) 2021                                                            **
** BY MEVION MEDICAL SYSTEMS, INC., LITTLETON, MA, USA.                          **
***********************************************************************************/

#include "CollisionChecker.h"
#include "CCVertices.h"
#include "CCMesh.h"
#include "CCTransform.h"
#include "CCChildManager.h"
#include "CCCollision.h"
#include "CCMeshLoader.h"
#include "CCGantryPositionSourceManual.h"
#include "CCCouchPositionSourceManual.h"
#include "CCTransformGantry.h"
#include "CCTransformRobotFrame.h"
#include "CCTransformScanner.h"
#include "CCUtil.h"
#include "RoboCouchModel.h"

namespace
{

/** @brief Tag indicating an object is not a shadow caster. */
const std::string NOT_SHADOW_CASTER("_NoShadow");

}

namespace mev
{

const int TABLE_FRAME_ID = 7;
const int FOREARM_FRAME_ID = 3;
const int UPPER_ARM_FRAME_ID = 2;

CC_DECLARE_COMPONENT(std::string, CC_OBJECT_NAME);
CC_DECLARE_COMPONENT(bool,        CC_OBJECT_INVISIBLE);
CC_DECLARE_COMPONENT(bool,        CC_OBJECT_SHADOW_CASTER);

/**
 * @brief Constructor.
 * @param roboCouch RoboCouchModel reference (needed for calculations).
 */
CollisionChecker::CollisionChecker(const RoboCouchModel& roboCouch) :
    m_roboCouch(roboCouch),
    m_registry(new CCRegistry()),
    m_childManager(new CCChildManager(m_registry)),
    m_collision(new CCCollision(m_registry)),
    m_gantryUserPositionSource(nullptr),
    m_couchUserPositionSource(nullptr)
{
    /* User controlled sources. */

    m_gantryUserPositionSource = new CCGantryPositionSourceManual();
    m_couchUserPositionSource = new CCCouchPositionSourceManual(m_roboCouch);

    /* Create objects. */

    m_objects.resize(CCObjectIdCount);
    for (size_t i = 0; i < CCObjectIdCount; ++i)
    {
        m_objects[i] = m_registry->create();
    }

    /* Set object names. */

    m_registry->attach(m_objects[StaticObjectsId],          CC_OBJECT_NAME, std::string("Static"));
    m_registry->attach(m_objects[ThetaId],                  CC_OBJECT_NAME, std::string("Theta"));
    m_registry->attach(m_objects[ExtensionId],              CC_OBJECT_NAME, std::string("Extension"));
    m_registry->attach(m_objects[TableId],                  CC_OBJECT_NAME, std::string("Table"));
    m_registry->attach(m_objects[ForearmId],                CC_OBJECT_NAME, std::string("Forearm"));
    m_registry->attach(m_objects[UpperArmId],               CC_OBJECT_NAME, std::string("UpperArm"));
    m_registry->attach(m_objects[ThetaUserId],              CC_OBJECT_NAME, std::string("ThetaUser"));
    m_registry->attach(m_objects[ExtensionUserId],          CC_OBJECT_NAME, std::string("ExtensionUser"));
    m_registry->attach(m_objects[TableUserId],              CC_OBJECT_NAME, std::string("TableUser"));
    m_registry->attach(m_objects[ForearmUserId],            CC_OBJECT_NAME, std::string("ForearmUser"));
    m_registry->attach(m_objects[UpperArmUserId],           CC_OBJECT_NAME, std::string("UpperArmUser"));
    m_registry->attach(m_objects[RingId],                   CC_OBJECT_NAME, std::string("Ring"));
    m_registry->attach(m_objects[SourceId],                 CC_OBJECT_NAME, std::string("Source"));
    m_registry->attach(m_objects[PanelId],                  CC_OBJECT_NAME, std::string("Panel"));
    m_registry->attach(m_objects[ScannerCoveringTubeId],    CC_OBJECT_NAME, std::string("ScannerCoveringTube"));

    /*
     * Mark objects that should not be visualized.
     */

    m_registry->attach(m_objects[ThetaUserId],              CC_OBJECT_INVISIBLE, true);
    m_registry->attach(m_objects[ExtensionUserId],          CC_OBJECT_INVISIBLE, true);
    m_registry->attach(m_objects[TableUserId],              CC_OBJECT_INVISIBLE, true);
    m_registry->attach(m_objects[ForearmUserId],            CC_OBJECT_INVISIBLE, true);
    m_registry->attach(m_objects[UpperArmUserId],           CC_OBJECT_INVISIBLE, true);
    m_registry->attach(m_objects[ScannerCoveringTubeId],    CC_OBJECT_INVISIBLE, true);

    /* User object positions. */

    m_registry->attach(
              m_objects[ThetaUserId],
              CC_OBJECT_TRANSFORM,
              tmp::shared_ptr<CCTransformGantry>(
                  new CCTransformGantry(m_gantryUserPositionSource,
                                        CCTransformGantry::PartTheta)));

    m_registry->attach(
              m_objects[ExtensionUserId],
              CC_OBJECT_TRANSFORM,
              tmp::shared_ptr<CCTransformGantry>(
                  new CCTransformGantry(m_gantryUserPositionSource,
                                        CCTransformGantry::PartExtension)));

    m_registry->attach(
              m_objects[TableUserId],
              CC_OBJECT_TRANSFORM,
              tmp::shared_ptr<CCTransformRobotFrame>(
                  new CCTransformRobotFrame(m_couchUserPositionSource, m_roboCouch, TABLE_FRAME_ID)));

    m_registry->attach(
              m_objects[ForearmUserId],
              CC_OBJECT_TRANSFORM,
              tmp::shared_ptr<CCTransformRobotFrame>(
                  new CCTransformRobotFrame(m_couchUserPositionSource, m_roboCouch, FOREARM_FRAME_ID)));

    m_registry->attach(
              m_objects[UpperArmUserId],
              CC_OBJECT_TRANSFORM,
              tmp::shared_ptr<CCTransformRobotFrame>(
                  new CCTransformRobotFrame(m_couchUserPositionSource, m_roboCouch, UPPER_ARM_FRAME_ID)));

    /*
     * Build the check lists. Each check will take some CPU time so it is better to add only
     * pairs of objects that can actually collide.
     */

    m_checkList.addPair(m_objects[TableId], m_objects[ThetaId]);
    m_checkList.addPair(m_objects[TableId], m_objects[ExtensionId]);
    m_checkList.addPair(m_objects[ForearmId], m_objects[ThetaId]);
    m_checkList.addPair(m_objects[ForearmId], m_objects[ExtensionId]);

    resetCachedPairsList();
}

/**
 * @brief Destructor.
 */
CollisionChecker::~CollisionChecker()
{
    setScannerPositionSource(tmp::shared_ptr<CCScannerPositionSource>());
    setGantryPositionSource(tmp::shared_ptr<CCGantryPositionSource>());
    setCouchPositionSource(tmp::shared_ptr<CCCouchPositionSource>());

    unloadGeometry();

    m_registry->detach(m_objects[ThetaUserId], CC_OBJECT_TRANSFORM);
    m_registry->detach(m_objects[ExtensionUserId], CC_OBJECT_TRANSFORM);
    m_registry->detach(m_objects[TableUserId], CC_OBJECT_TRANSFORM);
    m_registry->detach(m_objects[ForearmUserId], CC_OBJECT_TRANSFORM);
    m_registry->detach(m_objects[UpperArmUserId], CC_OBJECT_TRANSFORM);

    delete m_gantryUserPositionSource;
    m_gantryUserPositionSource = nullptr;
    delete m_couchUserPositionSource;
    m_couchUserPositionSource = nullptr;

    delete m_collision;
    m_collision = nullptr;
    delete m_childManager;
    m_childManager = nullptr;
    delete m_registry;
    m_registry = nullptr;
}

/**
 * @return Pointer to the object registry.
 */
CCRegistry* CollisionChecker::registry()
{
    return m_registry;
}

/**
 * @brief Functor used as callback while loading geometry data. Can be replaced with lambda after
 * switching to c++11.
 * @note BNL: Callback is preferable to lambda for organizational reasons.
 */
struct CCMeshLoaderCB
{
    void operator()(const CCMeshLoaderData& data) const
    {
        CCObject ro = registry->findByComponentValue(CC_OBJECT_NAME, data.name);
        if (!ro.isValid())
        {
            ro = registry->create();
            registry->attach(ro, CC_OBJECT_NAME, data.name);
        }

        tmp::shared_ptr<CCVertices> vertices = registry->getCopy(
                    ro, CC_OBJECT_VERTICES, tmp::shared_ptr<CCVertices>());
        if (!vertices)
        {
            vertices = tmp::shared_ptr<CCVertices>(new CCVertices());
            vertices->set(data.vertices);
            registry->attach(ro, CC_OBJECT_VERTICES, vertices);
        }
        else
        {
            vertices->set(data.vertices);
        }

        if (loadMeshes && !data.v.empty())
        {
            tmp::shared_ptr<CCMesh> mesh = registry->getCopy(
                        ro, CC_OBJECT_MESH, tmp::shared_ptr<CCMesh>());
            if (!mesh)
            {
                mesh = tmp::shared_ptr<CCMesh>(new CCMesh());
                #if defined(MEV_VXWORKS) || defined(MEV_VXWORKS7)
                mesh->set(data.v, data.vt, data.vn);
                #else
                mesh->set(data.v, data.vt, data.vn, data.mtl);
                #endif
                registry->attach(ro, CC_OBJECT_MESH, mesh);
            }
            else
            {
                #if defined(MEV_VXWORKS) || defined(MEV_VXWORKS7)
                mesh->set(data.v, data.vt, data.vn);
                #else
                mesh->set(data.v, data.vt, data.vn, data.mtl);
                #endif
            }
        }

        const std::vector<std::string> split = splitString(data.name, '.');
        if (split.size() > 1)
        {
            const std::string parentName = split.at(0);

            CCObject po = registry->findByComponentValue(CC_OBJECT_NAME, parentName);
            if (!po.isValid())
            {
                po = registry->create();
                registry->attach(po, CC_OBJECT_NAME, parentName);
            }

            CCChildManager::addChild(registry, po, ro);
        }

        // Everything casts shadows except those objects tagged with NOT_SHADOW_CASTER
        if(data.name.find(NOT_SHADOW_CASTER) != std::string::npos)
        {
            registry->attach(ro, CC_OBJECT_SHADOW_CASTER, false);
        }
        else
        {
            registry->attach(ro, CC_OBJECT_SHADOW_CASTER, true);
        }
    }

    CCRegistry* registry;
    bool loadMeshes;
};

/**
 * @brief Helper function that copies component value from one object to another.
 * @param registry Object registry.
 * @param destination Destination object.
 * @param source Source object.
 * @param c Component to copy.
 */
template <typename T>
static void copyCompomentValue(
        CCRegistry* registry, CCObject destination, CCObject source, CCComponent<T> c)
{
    T* pointer = registry->getPointer(source, c);
    if (pointer != nullptr)
    {
        registry->attach(destination, c, *pointer);
    }
}

/**
 * @brief Loads the geometry data from specified file.
 * @param objectsFile Path to a file.
 * @param loadMeshes true to load the data that is only required for visualisation,
 * false to only load collision data.
 * @return true on success, false otherwise.
 */
bool CollisionChecker::loadGeometry(const std::string& objectsFile, bool loadMeshes)
{
    CCMeshLoaderCB cb = {m_registry, loadMeshes};

    if (!CCMeshLoader::load(objectsFile, cb))
    {
        return false;
    }

    /* Copy some data. */

    copyCompomentValue(
                m_registry, m_objects[ThetaUserId], m_objects[ThetaId], CC_OBJECT_VERTICES);
    copyCompomentValue(
                m_registry, m_objects[ExtensionUserId], m_objects[ExtensionId], CC_OBJECT_VERTICES);
    copyCompomentValue(
                m_registry, m_objects[TableUserId], m_objects[TableId], CC_OBJECT_VERTICES);
    copyCompomentValue(
                m_registry, m_objects[ForearmUserId], m_objects[ForearmId], CC_OBJECT_VERTICES);
    copyCompomentValue(
                m_registry, m_objects[UpperArmUserId], m_objects[UpperArmId], CC_OBJECT_VERTICES);

    copyCompomentValue(
                m_registry, m_objects[ThetaUserId], m_objects[ThetaId], CC_OBJECT_MESH);
    copyCompomentValue(
                m_registry, m_objects[ExtensionUserId], m_objects[ExtensionId], CC_OBJECT_MESH);
    copyCompomentValue(
                m_registry, m_objects[TableUserId], m_objects[TableId], CC_OBJECT_MESH);
    copyCompomentValue(
                m_registry, m_objects[ForearmUserId], m_objects[ForearmId], CC_OBJECT_MESH);
    copyCompomentValue(
                m_registry, m_objects[UpperArmUserId], m_objects[UpperArmId], CC_OBJECT_MESH);

    resetCachedPairsList();

    return true;
}

/**
 * @brief Unloads the geometry data.
 */
void CollisionChecker::unloadGeometry()
{
    for (std::set<CCObject>::const_iterator
         it = m_registry->objects().begin(); it != m_registry->objects().end(); ++it)
    {
        m_registry->detachNoCheck(*it, CC_OBJECT_VERTICES);
        m_registry->detachNoCheck(*it, CC_OBJECT_MESH);
    }

    resetCachedPairsList();
}

/**
 * @brief Can be used to set the source that will be used to obtain the current gantry position.
 * @param source Gantry position source or nullptr to unset.
 */
void CollisionChecker::setGantryPositionSource(tmp::shared_ptr<CCGantryPositionSource> source)
{
    if (m_gantryPositionSource)
    {
        m_registry->detach(m_objects[ThetaId], CC_OBJECT_TRANSFORM);
        m_registry->detach(m_objects[ExtensionId], CC_OBJECT_TRANSFORM);
        resetCachedPairsList();
    }

    m_gantryPositionSource = source;

    if (m_gantryPositionSource)
    {
        m_registry->attach(
                    m_objects[ThetaId],
                    CC_OBJECT_TRANSFORM,
                    tmp::shared_ptr<CCTransformGantry>(
                        new CCTransformGantry(m_gantryPositionSource.get(),
                                              CCTransformGantry::PartTheta)));
        m_registry->attach(
                    m_objects[ExtensionId],
                    CC_OBJECT_TRANSFORM,
                    tmp::shared_ptr<CCTransformGantry>(
                        new CCTransformGantry(m_gantryPositionSource.get(),
                                              CCTransformGantry::PartExtension)));
        resetCachedPairsList();
    }
}

/**
 * @brief Can be used to set the source that will be used to obtain the current couch position.
 * @param source Couch position source or nullptr to unset.
 */
void CollisionChecker::setCouchPositionSource(tmp::shared_ptr<CCCouchPositionSource> source)
{
    if (m_couchPositionSource)
    {
        m_registry->detach(m_objects[TableId], CC_OBJECT_TRANSFORM);
        m_registry->detach(m_objects[ForearmId], CC_OBJECT_TRANSFORM);
        m_registry->detach(m_objects[UpperArmId], CC_OBJECT_TRANSFORM);
        resetCachedPairsList();
    }

    m_couchPositionSource = source;

    if (m_couchPositionSource)
    {
        m_registry->attach(
                  m_objects[TableId],
                  CC_OBJECT_TRANSFORM,
                  tmp::shared_ptr<CCTransformRobotFrame>(
                      new CCTransformRobotFrame(m_couchPositionSource.get(), m_roboCouch, TABLE_FRAME_ID)));
        m_registry->attach(
                  m_objects[ForearmId],
                  CC_OBJECT_TRANSFORM,
                  tmp::shared_ptr<CCTransformRobotFrame>(
                      new CCTransformRobotFrame(m_couchPositionSource.get(), m_roboCouch, FOREARM_FRAME_ID)));
        m_registry->attach(
                  m_objects[UpperArmId],
                  CC_OBJECT_TRANSFORM,
                  tmp::shared_ptr<CCTransformRobotFrame>(
                      new CCTransformRobotFrame(m_couchPositionSource.get(), m_roboCouch, UPPER_ARM_FRAME_ID)));
        resetCachedPairsList();
    }
}

/**
 * @brief Can be used to set the source that will be used to obtain the current scanner position.
 * @param source Scanner position source or nullptr to unset.
 */
void CollisionChecker::setScannerPositionSource(tmp::shared_ptr<CCScannerPositionSource> source)
{
    if (m_scannerPositionSource)
    {
        CCChildManager::detachRecursive(m_registry, m_objects[RingId], CC_OBJECT_TRANSFORM);
        m_registry->detach(m_objects[SourceId], CC_OBJECT_TRANSFORM);
        m_registry->detach(m_objects[PanelId], CC_OBJECT_TRANSFORM);
        m_registry->detach(m_objects[ScannerCoveringTubeId], CC_OBJECT_TRANSFORM);
        resetCachedPairsList();
    }

    m_scannerPositionSource = source;

    if (m_scannerPositionSource)
    {
        CCChildManager::attachRecursive(
                    m_registry,
                    m_objects[RingId],
                    CC_OBJECT_TRANSFORM,
                    tmp::shared_ptr<CCTransformScanner>(
                        new CCTransformScanner(m_scannerPositionSource.get(),
                                               CCTransformScanner::PartRing)));
        m_registry->attach(
                    m_objects[SourceId],
                    CC_OBJECT_TRANSFORM,
                    tmp::shared_ptr<CCTransformScanner>(
                        new CCTransformScanner(m_scannerPositionSource.get(),
                                               CCTransformScanner::PartSource)));
        m_registry->attach(
                    m_objects[PanelId],
                    CC_OBJECT_TRANSFORM,
                    tmp::shared_ptr<CCTransformScanner>(
                        new CCTransformScanner(m_scannerPositionSource.get(),
                                               CCTransformScanner::PartPanel)));
        m_registry->attach(
                    m_objects[ScannerCoveringTubeId],
                    CC_OBJECT_TRANSFORM,
                    tmp::shared_ptr<CCTransformScanner>(
                        new CCTransformScanner(m_scannerPositionSource.get(),
                                               CCTransformScanner::PartCoveringTube)));
        resetCachedPairsList();
    }
}

/**
 * @brief Adds the CT scanner to the check list.
 */
void CollisionChecker::enableScanner()
{
    m_checkList.addPair(m_objects[TableId], m_objects[RingId]);
    m_checkList.addPair(m_objects[TableId], m_objects[SourceId]);
    m_checkList.addPair(m_objects[TableId], m_objects[PanelId]);
    m_checkList.addPair(m_objects[ForearmId], m_objects[RingId]);
    m_checkList.addPair(m_objects[ForearmId], m_objects[SourceId]);
    m_checkList.addPair(m_objects[ForearmId], m_objects[PanelId]);

    m_checkList.addPair(m_objects[TableId], m_objects[ScannerCoveringTubeId]);
    m_checkList.addPair(m_objects[ForearmId], m_objects[ScannerCoveringTubeId]);

    resetCachedPairsList();
}

/**
 * @brief Can be used to print the positions that are currently returned by all position sources.
 */
void CollisionChecker::logState() const
{
    if (m_gantryPositionSource)
    {
        MEV_LOG_DEBUG("Gantry " << m_gantryPositionSource->get().second);
    }

    if (m_couchPositionSource)
    {
        MEV_LOG_DEBUG("Couch " << m_couchPositionSource->getJoints().second);
    }

    if (m_scannerPositionSource)
    {
        MEV_LOG_DEBUG("Scanner " << m_scannerPositionSource->get().second);
    }

    MEV_LOG_DEBUG("GantryUser " << m_gantryUserPositionSource->get().second);
    MEV_LOG_DEBUG("CouchUser " << m_couchUserPositionSource->getJoints().second);
}

/**
 * @brief Can be used to enable storing of the collision data. Currently only used
 * for visualisation.
 * @param enabled true to enable marking, false to disable.
 */
void CollisionChecker::setStoreCollisionData(bool enabled)
{
    m_collision->setStoreCollisionData(enabled);
}

/**
 * @brief Clears any stored collision data. Required mostly to "unmark" collisions in visualization.
 * @note Does not notify CCCollision listeners!
 * @see CCCollision::notifyCollisionData().
 */
void CollisionChecker::clearCollisionData()
{
    m_collision->clearCollisionData();
}

/**
 * @brief Update cached object pairs flat list
 */
void CollisionChecker::updateCheckPairsList()
{
    resetCachedPairsList();
    CheckResult& checkResult = m_checkPairsList.get();
    const CCCollision::CheckList pairs = m_collision->getCheckPairsList(m_checkList);

    for (std::vector<CCCollision::CheckList::Pair>::const_iterator it = pairs.pairs().begin();
         it != pairs.pairs().end();
         ++it)
    {
    	PairCheckData data;
    	data.pair = *it;
        checkResult.push_back(data);
    }

    m_checkPairsUserList = m_checkPairsList;
}

/**
 * @brief Reset (invalidate) cached object pairs flat list
 */
void CollisionChecker::resetCachedPairsList()
{
    m_checkPairsList.reset();
    m_checkPairsUserList.reset();
}

/**
 * @brief Checks collision between object pairs that are specified in the list.
 * @param list List of object pairs to check.
 * @note Does notify CCCollision listeners.
 * @see CCCollision::notifyCollisionData().
 */
void CollisionChecker::checkList(CheckResult& list)
{
    for (CheckResult::iterator it = list.begin();
         it != list.end();
         ++it)
    {
        PairCheckData& checkData = *it;
        checkData.previousResult = checkData.currentResult;
        checkData.currentResult = m_collision->check(checkData.pair.first, checkData.pair.second);
    #if CC_VERBOSITY >= 2
        MEV_LOG_DEBUG("Collision check result: " 
                <<  m_registry->getCopy(checkData.pair.first, CC_OBJECT_NAME, "") << " <-> "
                <<  m_registry->getCopy(checkData.pair.second, CC_OBJECT_NAME, "") 
                << checkData.currentResult);
    #endif
    
    #if CC_VERBOSITY >= 2
        logState();
    #endif
    }



    m_collision->notifyCollisionData();
}

/**
 * @brief Replaces one object with another in all list pairs.
 * @param first Object to replace.
 * @param second Object to replace with.
 */
void replaceObject(CachedList<CollisionChecker::PairCheckData>::ListType& list, CCObject first, CCObject second)
{
    for (CachedList<CollisionChecker::PairCheckData>::ListType::iterator it = list.begin(); it != list.end(); ++it)
    {
        if ((*it).pair.first == first)
        {
            (*it).pair.first = second;
        }
        if ((*it).pair.second == first)
        {
            (*it).pair.second = second;
        }
    }
}

/**
 * @brief Used to configure the user gantry position source and the user check list.
 * @param ud Gantry position to configure for.
 */
void CollisionChecker::configure(const UserData<GantryObjectId, CCGantryPosition>& ud)
{
    m_gantryUserPositionSource->set(ud.data());
    replaceObject(m_checkPairsUserList.get(), m_objects[ThetaId], m_objects[ThetaUserId]);
    replaceObject(m_checkPairsUserList.get(), m_objects[ExtensionId], m_objects[ExtensionUserId]);
}

/**
 * @brief Used to configure the user couch position source and the user check list.
 * @param ud Couch position to configure for.
 */
void CollisionChecker::configure(const UserData<CouchObjectId, CCCouchPositionIso>& ud)
{
    m_couchUserPositionSource->setIso(ud.data());
    replaceObject(m_checkPairsUserList.get(), m_objects[TableId], m_objects[TableUserId]);
    replaceObject(m_checkPairsUserList.get(), m_objects[ForearmId], m_objects[ForearmUserId]);
    replaceObject(m_checkPairsUserList.get(), m_objects[UpperArmId], m_objects[UpperArmUserId]);
}

/**
 * @brief Used to configure the user couch position source and the user check list.
 * @param ud Couch position to configure for.
 */
void CollisionChecker::configure(const UserData<CouchObjectId, CCCouchPositionJoints>& ud)
{
    m_couchUserPositionSource->setJoints(ud.data());
    replaceObject(m_checkPairsUserList.get(), m_objects[TableId], m_objects[TableUserId]);
    replaceObject(m_checkPairsUserList.get(), m_objects[ForearmId], m_objects[ForearmUserId]);
    replaceObject(m_checkPairsUserList.get(), m_objects[UpperArmId], m_objects[UpperArmUserId]);
}

/**
 * @brief Set scene object visibility.
 * @param objId Object.
 * @param visible true for visible, false otherwise.
 */
void CollisionChecker::setVisible(const CCObjectId objId, const bool visible)
{
    // Detaching to take advantage of automatic visibilty update on attach
    CCChildManager::detachRecursiveNoCheck(m_registry, m_objects[objId], CC_OBJECT_INVISIBLE);
    CCChildManager::attachRecursive(m_registry, m_objects[objId], CC_OBJECT_INVISIBLE, !visible);
}

} // namespace mev
