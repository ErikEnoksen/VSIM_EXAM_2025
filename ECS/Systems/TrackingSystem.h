#ifndef TRACKINGSYSTEM_H
#define TRACKINGSYSTEM_H

#include "../ECS/Entity/EntityManager.h"
#include "../../Core/Utility/gpuresourcemanager.h"

namespace bbl
{
class TrackingSystem
{
public:
    TrackingSystem(EntityManager* entityManager, GPUResourceManager* gpuResources);

    void update(float dt);

    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool isEnabled() const { return m_enabled; }

    bool didUpdateMesh() const { return m_didUpdateMesh; }
    void clearUpdateFlag() { m_didUpdateMesh = false; }

private:
    bool samplePosition(EntityID entity, Transform* transform, Tracking* tracking, float dt);  // Changed to bool
    void updateTraceMesh(EntityID entity, Tracking* tracking);

    EntityManager* m_entityManager = nullptr;
    GPUResourceManager* m_gpuResources = nullptr;
    bool m_enabled = true;
    bool m_didUpdateMesh = false;
    std::unordered_map<EntityID, EntityID> m_traceEntities;

};
}

#endif // TRACKINGSYSTEM_H
