#include "GameWorld.h"
#include "../Editor/MainWindow.h"

bbl::GameWorld::GameWorld()
{
    m_terrain = std::make_unique<Terrain>();
}


void bbl::GameWorld::Setup()
{
    // if (m_terrain->loadFromHeightmap("../../Assets/Textures/heightmap.jpg", 0.15f, 1.f, 0.0f))
    // {
    //     m_terrainLoaded = true;
    //     qDebug() << "Terrain loaded successfully!";
    // }
    // else
    // {
    //     qWarning() << "Failed to load terrain!";
    // }
                                                    //HeightScale, GridSpacing, HeightPlacement
    if (m_terrain->loadFromPointCloud("../../Lasdata/lasdata.txt", 1.5f, 2.25f, 0.0f))
    {
        m_terrainLoaded = true;
        qDebug() << "Terrain loaded successfully from point cloud!";
    }
    else
    {
        qWarning() << "Failed to load terrain!";
    }
    /*
     * Oppgave 2.3: Sett opp friksjons-sone
     *
     * Plasserer en høy-friksjons sone (μ = 0.8) i sentrum av terrenget
     * Størrelse: 20x20 meter
     */
    //glm::vec3 zoneCenter(0.0f, 0.0f, 0.0f);  // Sentrum av terrain
    //glm::vec2 zoneSize(30.0f, 30.0f);        // 20x20 meter
    //float highFriction = 0.f;                // Høy friksjon (normal er 0.25)

    //m_terrain->setFrictionZone(zoneCenter, zoneSize, highFriction);

    // Påfør rød farge på friction zone
    //m_terrain->applyFrictionZoneColors();

    m_terrainLoaded = true;

    qInfo() << "GameWorld setup complete with friction zone!";



}

void bbl::GameWorld::initializeSystems(EntityManager* entityManager)
{
    if (!entityManager) {
        qWarning() << "Cannot initialize systems: EntityManager is null!";
        return;
    }

    // Physics System
    m_physicsSystem = std::make_unique<PhysicsSystem>(entityManager, m_terrain.get());
    m_physicsSystem->setGravity(glm::vec3(0.0f, -9.81f, 0.0f));

    // Collision System
    m_collisionSystem = std::make_unique<CollisionSystem>(entityManager, m_terrain.get());
    m_collisionSystem->setTerrainCollisionEnabled(true);
    m_collisionSystem->setEntityCollisionEnabled(true);
}

void bbl::GameWorld::update(float dt)
{
    if (m_paused) {
        return;
    }

    if (m_collisionSystem) {
        m_collisionSystem->update(dt);
    }
    if (m_physicsSystem) {
        m_physicsSystem->update(dt);
    }

}
