#ifndef PHYSICSSYSTEM_H
#define PHYSICSSYSTEM_H

#include "../Game/Terrain.h"
#include "../../ECS/Entity/EntityManager.h"
#include <glm/glm.hpp>

namespace bbl
{
class PhysicsSystem
{
public:
    explicit PhysicsSystem(EntityManager* entityManager, Terrain* terrain = nullptr);

    void update(float deltaTime);
    void setGravity(const glm::vec3& gravity);
    const glm::vec3& getGravity() const;

    void setEntityManager(EntityManager* entityManager) {
        m_entityManager = entityManager;
    }

    void setTerrain(Terrain* terrain) { m_terrain = terrain; }

private:
    EntityManager* m_entityManager;
    Terrain* m_terrain;
    glm::vec3 m_gravity{0.0f, -9.81f, 0.0f};
};
}

#endif // PHYSICSSYSTEM_H
