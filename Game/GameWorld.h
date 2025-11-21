#ifndef GAMEWORLD_H
#define GAMEWORLD_H
#include "../Game/Terrain.h"
#include "../ECS/Components/Physics.h"
#include "../ECS/Components/CollisionSystem.h"
#include "../ECS/Entity/EntityManager.h"
#include "../ECS/Systems/TrackingSystem.h"
#include "../Core/Utility/gpuresourcemanager.h"
#include <memory>

class Renderer;

namespace bbl
{
class GameWorld
{
public:
    GameWorld();
    void Setup();
    void update(float dt);
    void setPaused(bool paused) { m_paused = paused; }
    bool isPaused() const { return m_paused; }

    Terrain* getTerrain() const { return m_terrain.get(); }
    bool isTerrainLoaded() const { return m_terrainLoaded; }

    void setTerrainEntity(EntityID terrainID) {
        if (m_collisionSystem) {
            m_collisionSystem->setTerrainEntity(terrainID);
        }
    }

    void initializeSystems(EntityManager* entityManager, GPUResourceManager* gpuResources, Renderer* renderer);


    bool needsCommandBufferRebuild() const { return m_commandBuffersNeedRebuild; }
    void clearRebuildFlag() { m_commandBuffersNeedRebuild = false; }

private:
    std::unique_ptr<Terrain> m_terrain;
    std::unique_ptr<PhysicsSystem> m_physicsSystem;
    std::unique_ptr<CollisionSystem> m_collisionSystem;
    std::unique_ptr<bbl::TrackingSystem> m_trackingSystem;
    Renderer* m_renderer = nullptr;
    bool m_paused = true;
    bool m_terrainLoaded{false};
    bool m_commandBuffersNeedRebuild = false;
};
} // namespace bbl
#endif // GAMEWORLD_H
