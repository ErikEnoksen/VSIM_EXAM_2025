#include "Physics.h"
#include "../../Game/Terrain.h"
#include <QDebug>

using namespace bbl;

PhysicsSystem::PhysicsSystem(EntityManager* entityManager, Terrain* terrain)
    : m_entityManager(entityManager), m_terrain(terrain)
{
}

void PhysicsSystem::update(float dt)
{
    if (!m_entityManager) {
        return;
    }

    std::vector<bbl::EntityID> physicsEntities =
        m_entityManager->getEntitiesWith<bbl::Physics, bbl::Transform>();

    for (EntityID entity : physicsEntities) {
        bbl::Physics* physics = m_entityManager->getComponent<bbl::Physics>(entity);
        bbl::Transform* transform = m_entityManager->getComponent<bbl::Transform>(entity);

        if (!physics || !transform) {
            continue;
        }

        if (!physics->useGravity) {
            physics->velocity = glm::vec3(0.0f);
            physics->acceleration = glm::vec3(0.0f);
            continue;
        }

        bbl::Collision* collision = m_entityManager->getComponent<bbl::Collision>(entity);

        /*
         * Oppgave 2.1: Simulering av ball som ruller på terreng
         * Bruker Newtons andre lov (F = ma) og algoritme 9.6
         */
        if (m_terrain && physics->useGravity)
        {
            /*
             * Steg 1: Identifiser hvilken trekant ballen er på
             */
            float groundY = m_terrain->getHeightAt(transform->position.x,
                                                   transform->position.z,
                                                   glm::vec3(0.0f));

            /*
             * Steg 2: Beregn normalvektoren
             */
            glm::vec3 normal = m_terrain->getNormal(transform->position);
            normal = glm::normalize(normal);

            /*
             * Steg 3: Beregn akselerasjonsvektoren
             *
             * Hvis ballen er OVER terrenget, bruk vanlig tyngdekraft
             * Hvis ballen er PÅ terrenget, bruk tangent-akselerasjon
             */
            float ballRadius = 1.0f;  // Antar radius = 1m
            if (collision) {
                ballRadius = collision->colliderSize.y * 0.5f;
            }

            // Sjekk om ballen er over bakken
            if (transform->position.y > groundY + ballRadius + 0.5f) {
                // FRITT FALL - bruk full tyngdekraft
                physics->acceleration = glm::vec3(0.0f, -9.81f, 0.0f);
            }
            else {
                // PÅ TERRENGET - bruk tangent-akselerasjon
                glm::vec3 gravity(0.0f, -9.81f, 0.0f);
                float gravityDotNormal = glm::dot(gravity, normal);
                glm::vec3 tangentAcceleration = gravity - (gravityDotNormal * normal);

                physics->acceleration = tangentAcceleration;
            }
        }
        else {
            // Vanlig gravity hvis ikke på terrain
            if (physics->useGravity && collision && !collision->isGrounded)
            {
                physics->acceleration += m_gravity;
            }
            else if (collision && collision->isGrounded) {
                physics->acceleration.y = 0.0f;
                physics->velocity.y = 0.0f;
            }
        }

        /*
         * Steg 4: Oppdater hastighet (Formel 9.16)
         */
        physics->velocity += physics->acceleration * dt;

        /*
         * Steg 5: Oppdater posisjon (Formel 9.17)
         */
        transform->position += physics->velocity * dt;

        // Reset akselerasjon
        physics->acceleration = glm::vec3(0.0f);
    }
}

void PhysicsSystem::setGravity(const glm::vec3& gravity) {
    m_gravity = gravity;
}

const glm::vec3& PhysicsSystem::getGravity() const {
    return m_gravity;
}
