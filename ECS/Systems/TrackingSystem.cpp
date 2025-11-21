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

        // Sjekk om vi faktisk samplet et nytt punkt
        bool sampledNewPoint = false;

        // Sample posisjon ved faste intervaller
        sampledNewPoint = samplePosition(entity, transform, tracking, dt);

        // Oppdater mesh kun hvis vi samplet et nytt punkt OG har nok punkter
        if (sampledNewPoint && tracking->controlPoints.size() >= 2) {
            updateTraceMesh(entity, tracking);
        }
    }
}

bool TrackingSystem::samplePosition(EntityID entity, Transform* transform, Tracking* tracking, float dt)
{
    /*
     * Oppgave 2.5: "Sampling positions for fixed time intervals"
     *
     * Kontinuerlig bane: x(t)
     * Vi diskretiserer denne ved å sample posisjon ved faste tidsteg Δt:
     *
     *      t_k   = k · Δt          (k = 0,1,2,...)
     *      P_k   = x(t_k)          (kontrollpunkter / samplede posisjoner)
     *
     * I koden:
     *  - sampleInterval  ≈ Δt
     *  - timeSinceLastSample akkumulerer tid inntil vi når Δt
     */

    tracking->timeSinceLastSample += dt;   // dt = tidssteg per frame

    // Only sample when we reach the interval
    if (tracking->timeSinceLastSample >= tracking->sampleInterval) {
        // Vi har nå passert neste t_k
        tracking->timeSinceLastSample = 0.0f;

        // Nytt kontrollpunkt P_k = x(t_k) ≈ transform->position
        tracking->controlPoints.push_back(transform->position);

        // Begrens antall lagrede punkter (FIFO):
        // hvis |P_k| > maxPoints, fjern eldste P_0
        if (tracking->controlPoints.size() > static_cast<size_t>(tracking->maxPoints)) {
            tracking->controlPoints.erase(tracking->controlPoints.begin());
        }

        // Return true - vi samplet et nytt punkt
        return true;
    }

    // If interval not reached, just return without adding a point!
    return false;
}

void TrackingSystem::updateTraceMesh(EntityID entity, Tracking* tracking)
{
    /*
     * Oppgave 2.5: "Draw the ball's trace on the surface"
     *
     * Faglig ønsket modell (3D B-spline-kurve):
     * ----------------------------------------
     * En kubisk, uniform B-spline kan skrives som
     *
     *      C(t) = Σ_{i=0}^{n} N_{i,3}(t) · P_i
     *
     * der
     *      P_i           = kontrollpunkter (samplede posisjoner)
     *      N_{i,3}(t)    = B-spline basisfunksjoner av grad 3
     *
     * For ett kurvesegment mellom P0, P1, P2, P3 kan man også bruke matriseform:
     *
     *      C(u) = [u^3  u^2  u  1] · (1/6) · M_B · [P0  P1  P2  P3]^T,
     *      0 ≤ u ≤ 1,
     *
     * der M_B er B-spline-matrisen
     *
     *      M_B = [ -1  3 -3  1
     *               3 -6  3  0
     *              -3  0  3  0
     *               1  4  1  0 ].
     *
     * I denne implementasjonen (for enkelhets skyld):
     * ----------------------------------------------
     * Vi tegner foreløpig en polylinje (piecewise lineær approksimasjon)
     * gjennom kontrollpunktene:
     *
     *  For hvert intervall [P_i, P_{i+1}] kan en lineær interpolasjon skrives som
     *
     *      C_i(α) = (1 - α) · P_i + α · P_{i+1},   0 ≤ α ≤ 1.
     *
     * Når vi tegner en line strip med verteksene P_0, P_1, ..., P_n,
     * får vi geometrisk akkurat denne lineære kurven.
     */

    if (tracking->controlPoints.size() < 2) {
        return;
    }

    // Finn eller lag trace entity knyttet til denne ballen
    EntityID traceEntity = INVALID_ENTITY;
    auto it = m_traceEntities.find(entity);

    if (it == m_traceEntities.end()) {
        // Lag ny trace entity første gang
        traceEntity = m_entityManager->createEntity();
        m_traceEntities[entity] = traceEntity;

        // Legg til Transform (identitet – trace tegnes i world space)
        Transform traceTransform;
        traceTransform.position = glm::vec3(0.0f);
        m_entityManager->addComponent(traceEntity, traceTransform);
    } else {
        traceEntity = it->second;
    }

    // Lag mesh-data fra kontrollpunktene P_k
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    /*
     * Oppgave 2.5: Line strip mesh
     *
     * Vi bruker hvert kontrollpunkt P_k som en vertex i et line strip.
     * GPU-en tegner så rette linjer mellom P_k og P_{k+1}.
     */
    for (size_t i = 0; i < tracking->controlPoints.size(); ++i) {
        Vertex v;
        v.pos      = tracking->controlPoints[i];         // P_k = (x_k, y_k, z_k)
        v.color    = glm::vec3(1.0f, 0.0f, 0.0f);        // rød linje for trace
        v.texCoord = glm::vec2(0.0f);
        vertices.push_back(v);

        // Lag line-indekser mellom P_i og P_{i+1}
        if (i < tracking->controlPoints.size() - 1) {
            indices.push_back(static_cast<uint32_t>(i));
            indices.push_back(static_cast<uint32_t>(i + 1));
        }
    }

    if (vertices.empty() || indices.empty()) {
        return;
    }

    // Sjekk om mesh allerede eksisterer
    Mesh* meshComp = m_entityManager->getComponent<Mesh>(traceEntity);

    if (meshComp && meshComp->meshResourceID != 0) {
        // Frigjør gammel mesh før vi laster opp ny
        m_gpuResources->releaseMeshResources(meshComp->meshResourceID);
    }

    // Upload mesh til GPU slik at vi kan tegne linjen i Vulkan
    MeshData meshData;
    meshData.vertices = vertices;
    meshData.indices  = indices;
    size_t meshResourceID = m_gpuResources->uploadMesh(meshData);

    // Oppdater eller legg til Mesh-komponent
    if (meshComp) {
        meshComp->meshResourceID = meshResourceID;
    } else {
        Mesh newMesh;
        newMesh.meshResourceID = meshResourceID;
        m_entityManager->addComponent(traceEntity, newMesh);
    }

    // Oppdater eller legg til Render-komponent
    Render* renderComp = m_entityManager->getComponent<Render>(traceEntity);
    if (renderComp) {
        renderComp->meshResourceID = meshResourceID;
        renderComp->visible        = true;
        renderComp->usePhong       = false;
        renderComp->isLine         = true;
    } else {
        Render newRender;
        newRender.meshResourceID = meshResourceID;
        newRender.visible        = true;
        newRender.usePhong       = false;
        newRender.isLine         = true;
        m_entityManager->addComponent(traceEntity, newRender);
    }
    m_didUpdateMesh = true;
}
