#include "CollisionSystem.h"
#include <algorithm>
#include <qdebug.h>

using namespace bbl;

CollisionSystem::CollisionSystem(EntityManager* entityManager, Terrain* terrain)
    : m_entityManager(entityManager)
    , m_terrain(terrain)
{
}

void CollisionSystem::update(float dt)
{
    if (!m_entityManager) {
        return;
    }

    // Get all entities with collision components
    std::vector<EntityID> collisionEntities = m_entityManager->getEntitiesWith<Collision, Transform>();

    // Reset collision (blud)
    for (EntityID entity : collisionEntities) {
        Collision* collision = m_entityManager->getComponent<Collision>(entity);
        if (collision) {
            collision->isGrounded = false;
            collision->isColliding = false;
        }
    }

    // Check terrain collisions first(Before rest)
    if (m_terrainCollisionEnabled && m_terrain) {
        for (EntityID entity : collisionEntities) {
            Transform* transform = m_entityManager->getComponent<Transform>(entity);
            Collision* collision = m_entityManager->getComponent<Collision>(entity);

            if (transform && collision) {
                checkTerrainCollision(entity, transform, collision);
            }
        }
    }

    if (m_entityCollisionEnabled) {
        checkEntityCollisions();
    }
}

AABB CollisionSystem::calculateAABB(const Transform& transform, const Collision& collision) const
{
    glm::vec3 halfSize = collision.colliderSize * 0.5f * transform.scale;

    AABB aabb;
    aabb.min = transform.position - halfSize;
    aabb.max = transform.position + halfSize;

    return aabb;
}


void CollisionSystem::checkTerrainCollision(EntityID entity, Transform* transform, Collision* collision)
{
    if (!m_terrain || !transform || !collision) {
        return;
    }

    glm::vec3 terrainPosition(0.0f);
    if (m_terrainEntityID != INVALID_ENTITY) {
        if (auto* terrainTransform = m_entityManager->getComponent<Transform>(m_terrainEntityID)) {
            terrainPosition = terrainTransform->position;
        }
    }

    float terrainHeight = m_terrain->getHeightAt(transform->position.x, transform->position.z, terrainPosition);

    float colliderHalfHeight = (collision->colliderSize.y * transform->scale.y) * 0.5f;
    float entityBottom = transform->position.y - colliderHalfHeight;

    if (entityBottom <= terrainHeight + 0.1f) {
        collision->isGrounded = true;
        collision->isColliding = true;

        // Snap entity to terrain surface
        if (entityBottom < terrainHeight) {
            transform->position.y = terrainHeight + colliderHalfHeight;
        }

        Physics* physics = m_entityManager->getComponent<Physics>(entity);
        if (physics && physics->velocity.y < -0.1f) {
            // Reverser Y-hastighet med restitusjon
            float restitution = 0.1f;
            physics->velocity.y = -physics->velocity.y * restitution;

            // La til Bouce for mer realisisk simulasjon
            if (std::abs(physics->velocity.y) < 0.1f) {
                physics->velocity.y = 0.0f;
            }
        }
    }
}
void CollisionSystem::checkEntityCollisions()
{
    if (!m_entityManager) {
        return;
    }

    std::vector<EntityID> collisionEntities = m_entityManager->getEntitiesWith<Collision, Transform>();

    // Check all pairs of entities
    for (size_t i = 0; i < collisionEntities.size(); ++i) {
        EntityID entityA = collisionEntities[i];
        Transform* transformA = m_entityManager->getComponent<Transform>(entityA);
        Collision* collisionA = m_entityManager->getComponent<Collision>(entityA);

        if (!transformA || !collisionA) {
            continue;
        }

        AABB aabbA = calculateAABB(*transformA, *collisionA);

        for (size_t j = i + 1; j < collisionEntities.size(); ++j) {
            EntityID entityB = collisionEntities[j];
            Transform* transformB = m_entityManager->getComponent<Transform>(entityB);
            Collision* collisionB = m_entityManager->getComponent<Collision>(entityB);

            if (!transformB || !collisionB) {
                continue;
            }

            AABB aabbB = calculateAABB(*transformB, *collisionB);

            // Check  AABBs overlap
            if (aabbA.intersects(aabbB)) {
                collisionA->isColliding = true;
                collisionB->isColliding = true;


                if (collisionA->isTrigger || collisionB->isTrigger) {
                    continue;
                }


                resolveCollision(entityA, entityB, transformA, transformB);
            }
        }
    }
}

void CollisionSystem::resolveCollision(EntityID entityA, EntityID entityB,
                                       Transform* transformA, Transform* transformB)
{
    /*
     * Oppgave 2.4: Kollisjon
     */

    glm::vec3 centerA = transformA->position;
    glm::vec3 centerB = transformB->position;
    glm::vec3 delta = centerB - centerA;

    Collision* collisionA = m_entityManager->getComponent<Collision>(entityA);
    Collision* collisionB = m_entityManager->getComponent<Collision>(entityB);

    glm::vec3 halfSizeA = collisionA->colliderSize * 0.5f * transformA->scale;
    glm::vec3 halfSizeB = collisionB->colliderSize * 0.5f * transformB->scale;
    glm::vec3 totalHalfSize = halfSizeA + halfSizeB;

    float overlapX = totalHalfSize.x - std::abs(delta.x);
    float overlapY = totalHalfSize.y - std::abs(delta.y);
    float overlapZ = totalHalfSize.z - std::abs(delta.z);

    glm::vec3 separation(0.0f);

    if (overlapX < overlapY && overlapX < overlapZ) {
        separation.x = (delta.x > 0) ? overlapX : -overlapX;
    } else if (overlapY < overlapZ) {
        separation.y = (delta.y > 0) ? overlapY : -overlapY;
    } else {
        separation.z = (delta.z > 0) ? overlapZ : -overlapZ;
    }

    Physics* physicsA = m_entityManager->getComponent<Physics>(entityA);
    Physics* physicsB = m_entityManager->getComponent<Physics>(entityB);

    /*
     * Separasjon
     */
    if (physicsA && !physicsB) {
        transformA->position -= separation;
    }
    else if (!physicsA && physicsB) {
        transformB->position += separation;
    }
    else if (physicsA && physicsB) {
        transformA->position -= separation * 0.5f;
        transformB->position += separation * 0.5f;
    }

    /*
     * Hastighetskorrigering
     */
    if (glm::length(separation) < 0.001f) {
        return;
    }

    glm::vec3 normal = glm::normalize(separation);

    /*
     * Oppgave 2.4: Ball mot statisk objekt (cube)
     * Behandle som ball-ball med uendelig masse
     */
    if (physicsA && !physicsB) {
        // A er ball, B er statisk cube
        // Hastighet langs kollisjonsretningen
        float v0_d = glm::dot(physicsA->velocity, normal);

        // Kun korriger hvis ballen beveger seg MOT cuben
        if (v0_d < 0.0f) {
            // Kapittel 9.7.5: Med uendelig masse på B:
            // v'₀ = -v₀ (perfekt refleksjon i normal-retningen)

            // Tangential hastighet bevares
            glm::vec3 v0_tangent = physicsA->velocity - v0_d * normal;

            //bevar tangent, reverser normal
            physicsA->velocity = v0_tangent - v0_d * normal;

            // Energitap og tangential friksjon
            physicsA->velocity *= 0.8f;  // 80% energi bevares

            // Ekstra tangential friksjon (glir mindre langs veggen)
            glm::vec3 tangentVel = physicsA->velocity - glm::dot(physicsA->velocity, normal) * normal;
            physicsA->velocity -= tangentVel * 0.3f;  // 30% tangential tap
        }
    }
    else if (!physicsA && physicsB) {
        // B er ball, A er statisk
        float v1_d = glm::dot(physicsB->velocity, normal);

        if (v1_d > 0.0f) {
            glm::vec3 v1_tangent = physicsB->velocity - v1_d * normal;
            physicsB->velocity = v1_tangent - v1_d * normal;
            physicsB->velocity *= 0.8f;

            glm::vec3 tangentVel = physicsB->velocity - glm::dot(physicsB->velocity, normal) * normal;
            physicsB->velocity -= tangentVel * 0.3f;
        }
    }
    else if (physicsA && physicsB) {
        /*
         * Oppgave 2.4: Ball-ball kollisjon
         * Kapittel 9.7.5: Formel 9.25 og 9.26
         */

        float m0 = physicsA->mass;
        float m1 = physicsB->mass;
        float totalMass = m0 + m1;

        float v0_d = glm::dot(physicsA->velocity, normal);
        float v1_d = glm::dot(physicsB->velocity, normal);

        // Kapittel 9.7.5: Formel 9.25 og 9.26
        float v0_d_new = ((m0 - m1) / totalMass) * v0_d + (2.0f * m1 / totalMass) * v1_d;
        float v1_d_new = ((m1 - m0) / totalMass) * v1_d + (2.0f * m0 / totalMass) * v0_d;

        glm::vec3 v0_tangent = physicsA->velocity - v0_d * normal;
        glm::vec3 v1_tangent = physicsB->velocity - v1_d * normal;

        // Kapittel 9.7.5: Formel 9.27 og 9.28
        physicsA->velocity = v0_tangent + v0_d_new * normal;
        physicsB->velocity = v1_tangent + v1_d_new * normal;

        physicsA->velocity *= 0.95f;
        physicsB->velocity *= 0.95f;
    }
}
