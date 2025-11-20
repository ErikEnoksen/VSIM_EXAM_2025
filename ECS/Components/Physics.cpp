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
             */
            float ballRadius = 1.0f;
            if (collision) {
                ballRadius = collision->colliderSize.y * 0.5f;
            }

            if (transform->position.y > groundY + ballRadius + 0.5f) {
                physics->acceleration = glm::vec3(0.0f, -9.81f, 0.0f);
            }
            else {
                glm::vec3 gravity(0.0f, -9.81f, 0.0f);
                float gravityDotNormal = glm::dot(gravity, normal);
                glm::vec3 tangentAcceleration = gravity - (gravityDotNormal * normal);

                physics->acceleration = tangentAcceleration;

                /*
                 * Oppgave 2.2/2.3 - Friksjon
                 */
                if(collision && collision->isGrounded)
                {
                    float currentSpeed = glm::length(physics->velocity);

                    if (currentSpeed > 0.01f) {
                        float my = m_terrain->getFrictionAt(transform->position.x, transform->position.z);

                        glm::vec3 up(0.0f, 1.0f, 0.0f);
                        float cosTheta = glm::dot(normal, up);

                        float g = 9.81f;
                        float frictionMagnitude = my * g * cosTheta;

                        glm::vec3 velocityDir = glm::normalize(physics->velocity);
                        glm::vec3 frictionAccel = -frictionMagnitude * velocityDir;

                        physics->acceleration += frictionAccel;
                    }
                }
            }
        }
        else {
            if (physics->useGravity && collision && !collision->isGrounded)
            {
                physics->acceleration += m_gravity;
            }
        }

        /*
         * Steg 4: Oppdater hastighet (Formel 9.16)
         */
        physics->velocity += physics->acceleration * dt;

        if (collision && collision->isGrounded) {
            float currentSpeed = glm::length(physics->velocity);

            if (currentSpeed < 0.01f) {
                physics->velocity = glm::vec3(0.0f);
            }
        }

        /*
         * Steg 5: Oppdater posisjon (Formel 9.17)
         */
        glm::vec3 oldPosition = transform->position;
        transform->position += physics->velocity * dt;

        /*
         * Steg 6: Beregn ballens rotasjonsvektor (Formel 9.11)
         * Steg 7: Beregn ballens rotasjon (Formel 9.10)
         *
         * Kapittel 9.4: "Ved rulling er det en sammenheng mellom
         * rotasjonsvinkel Θ, radius r og translasjon s"
         */
        if (collision && collision->isGrounded) {
            float ballRadius = collision->colliderSize.y * 0.5f;

            if (ballRadius > 0.0f && glm::length(physics->velocity) > 0.001f) {
                // Kapittel 9.4, Formel 9.10: Θ = s/r
                // s (translasjon) = distanse beveget dette framesteget
                glm::vec3 movement = transform->position - oldPosition;
                float distance = glm::length(movement);

                // Rotasjonsvinkel i radianer
                float rotationAngle = distance / ballRadius;

                // Kapittel 9.4, Formel 9.11: ~r = ~n × ~v
                // Rotasjonsaksen er vinkelrett på både normal og hastighet
                glm::vec3 terrainNormal = m_terrain->getNormal(transform->position);
                glm::vec3 velocityDir = glm::normalize(physics->velocity);
                glm::vec3 rotationAxis = glm::cross(terrainNormal, velocityDir);

                if (glm::length(rotationAxis) > 0.001f) {
                    rotationAxis = glm::normalize(rotationAxis);

                    // Oppdater rotasjon (akkumulert over tid)
                    // Roter rundt rotationAxis med rotationAngle
                    transform->rotation += rotationAxis * rotationAngle;
                }
            }
        }

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
