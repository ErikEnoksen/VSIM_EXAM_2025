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

private:
    void samplePosition(EntityID entity, Transform* transform, Tracking* tracking, float dt);
    void updateTraceMesh(EntityID entity, Tracking* tracking);

    EntityManager* m_entityManager = nullptr;
    GPUResourceManager* m_gpuResources = nullptr;
    bool m_enabled = true;

    // Map: tracked entity → trace entity
    std::unordered_map<EntityID, EntityID> m_traceEntities;
};
}

#endif // TRACKINGSYSTEM_H
