#include "TrackingSystem.h"
#include <QDebug>

using namespace bbl;

TrackingSystem::TrackingSystem(EntityManager* entityManager, GPUResourceManager* gpuResources)
    : m_entityManager(entityManager)
    , m_gpuResources(gpuResources)
{
}

void TrackingSystem::update(float dt)
{
    if (!m_entityManager || !m_enabled) {
        return;
    }

    /*
     * Oppgave 2.5: Tracking/visualisering av ballens trace
     */

    std::vector<EntityID> trackingEntities =
        m_entityManager->getEntitiesWith<Tracking, Transform>();

    for (EntityID entity : trackingEntities) {
        Tracking* tracking = m_entityManager->getComponent<Tracking>(entity);
        Transform* transform = m_entityManager->getComponent<Transform>(entity);

        if (!tracking || !transform || !tracking->enabled) {
            continue;
        }

        // Sample posisjon ved faste intervaller
        samplePosition(entity, transform, tracking, dt);

        // Oppdater mesh hvis vi har nok punkter
        if (tracking->controlPoints.size() >= 2) {
            updateTraceMesh(entity, tracking);
        }
    }
}

void TrackingSystem::samplePosition(EntityID entity, Transform* transform, Tracking* tracking, float dt)
{
    /*
     * Oppgave 2.5: "Sampling positions for fixed time intervals"
     */

    tracking->timeSinceLastSample += dt;

    if (tracking->timeSinceLastSample >= tracking->sampleInterval) {
        tracking->timeSinceLastSample = 0.0f;

        // Legg til ny posisjon
        tracking->controlPoints.push_back(transform->position);

        // Fjern eldste punkter hvis vi har for mange
        if (tracking->controlPoints.size() > static_cast<size_t>(tracking->maxPoints)) {
            tracking->controlPoints.erase(tracking->controlPoints.begin());
        }

        qDebug() << "Sampled position:" << transform->position.x << transform->position.y << transform->position.z
                 << "Total points:" << tracking->controlPoints.size();
    }
}

void TrackingSystem::updateTraceMesh(EntityID entity, Tracking* tracking)
{
    /*
     * Oppgave 2.5: "Draw the ball's trace on the surface"
     *
     */

    if (tracking->controlPoints.size() < 2) {
        return;
    }

    // Finn eller lag trace entity
    EntityID traceEntity = INVALID_ENTITY;

    auto it = m_traceEntities.find(entity);
    if (it == m_traceEntities.end()) {

        // Lag ny trace entity første gang
        traceEntity = m_entityManager->createEntity();
        m_traceEntities[entity] = traceEntity;

        // Legg til Transform
        Transform traceTransform;
        traceTransform.position = glm::vec3(0.0f);
        m_entityManager->addComponent(traceEntity, traceTransform);

        qInfo() << "Created trace entity" << traceEntity << "for entity" << entity;
    } else {
        traceEntity = it->second;
    }

    // Lag mesh data fra kontrollpunkter
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    /*
     * Oppgave 2.5: Lag linje-strip mesh
     * Rette linjer mellom kontrollpunkter
     */
    for (size_t i = 0; i < tracking->controlPoints.size(); ++i) {
        Vertex v;
        v.pos = tracking->controlPoints[i];
        v.color = glm::vec3(1.0f, 0.0f, 0.0f);  // RØD
        v.texCoord = glm::vec2(0.0f);
        vertices.push_back(v);

        // Lag line strip indices (linje mellom punkt i og i+1)
        if (i < tracking->controlPoints.size() - 1) {
            indices.push_back(static_cast<uint32_t>(i));
            indices.push_back(static_cast<uint32_t>(i + 1));
        }
    }

    // Upload mesh til GPU for å kunne runne linjen. kan ikke gjøre av GPU
    if (!vertices.empty() && !indices.empty()) {

        MeshData meshData;
        meshData.vertices = vertices;
        meshData.indices = indices;

        size_t meshResourceID = m_gpuResources->uploadMesh(meshData);

        // Oppdater eller legg til Mesh komponent
        Mesh* meshComp = m_entityManager->getComponent<Mesh>(traceEntity);
        if (meshComp) {
            meshComp->meshResourceID = meshResourceID;
        } else {
            Mesh newMesh;
            newMesh.meshResourceID = meshResourceID;
            m_entityManager->addComponent(traceEntity, newMesh);
        }

        // Oppdater eller legg til Render komponent
        Render* renderComp = m_entityManager->getComponent<Render>(traceEntity);
        if (renderComp) {
            renderComp->meshResourceID = meshResourceID;
            renderComp->visible = true;
            renderComp->usePhong = false;
        } else {
            Render newRender;
            newRender.meshResourceID = meshResourceID;
            newRender.visible = true;
            newRender.usePhong = false;
            newRender.isLine = true;
            m_entityManager->addComponent(traceEntity, newRender);
        }
    }
}
