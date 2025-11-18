#include "../Game/Terrain.h"
#include "../External/stb_image.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <unordered_map>
#include <limits>
#include <QDebug>

Terrain::Terrain() : m_width (0), m_height(0), m_channels(0), m_heightScale(0.02f), m_gridSpacing(0.2f), m_heightPlacement(-5.0f)
{

}

Terrain::~Terrain()
{
}


bool Terrain::loadFromHeightmap(const std::string& filepath, float heightScale, float gridSpacing,float heightPlacement)
{
    m_heightScale = heightScale;
    m_gridSpacing = gridSpacing;
    m_heightPlacement = heightPlacement;

    stbi_uc* pixelData = stbi_load(filepath.c_str(), &m_width, &m_height, &m_channels, STBI_rgb_alpha);

    if (pixelData == nullptr)
    {
        std::cerr << "Failed to load heightmap image: " << filepath << std::endl;
        return false;
    }

    std::cout << "Loaded heightmap: " << m_width << "x" << m_height
              << " (" << m_channels << " channels)" << std::endl;

    // Generate the mesh from pixel data
    generateMesh(pixelData);

    // Free the image data
    stbi_image_free(pixelData);

    std::cout << "Terrain generated: " << m_vertices.size() << " vertices, "
              << m_indices.size() << " indices" << std::endl;

    return true;
}


void Terrain::generateMesh(unsigned char* textureData)
{
    m_vertices.clear();
    m_indices.clear();
    m_heightData.clear();

    //Reserving space for vertices and heightData
    m_vertices.reserve(m_width * m_height);
    m_heightData.reserve(m_width * m_height);

    float vertexXStart = 0.0f - m_width * m_gridSpacing / 2.0f;
    float vertexZStart = 0.0f + m_height * m_gridSpacing / 2.0f;

    for (int d = 0; d < m_height; ++d)
    {
        for (int w = 0; w < m_width; ++w)
        {
            //Calculating index for R in RGBA
            int index = (w + d * m_width) * 4;

            //Get Height from red Channel
            float heightFromBitmap = static_cast<float>(textureData[index]) * m_heightScale + m_heightPlacement;

            //Storing height for Collision Detection
            m_heightData.push_back(heightFromBitmap);

            Vertex vertex{};

            //Position
            vertex.pos = glm::vec3(vertexXStart+ (w * m_gridSpacing), heightFromBitmap, vertexZStart - (d * m_gridSpacing));

            //Auto Coloring Will be replaced with texture
            float normalizedHeight = (heightFromBitmap - m_heightPlacement) / (255.0f * m_heightScale);
            vertex.color = glm::vec3(normalizedHeight, normalizedHeight * 0.8f, normalizedHeight * 0.6f);

            vertex.texCoord = glm::vec2(w / (m_width - 1.0f),d / (m_height - 1.0f));

            m_vertices.push_back(vertex);
        }
    }


    //Making indices two triangles per quad
    //Drawing the triangles from bottom left to top right
    m_indices.reserve((m_width - 1) * (m_height - 1) * 6);

    for (int d = 0; d < m_height - 1; ++d)
    {
        for (int w = 0; w < m_width - 1; ++w)
        {
            uint32_t topLeft = w + d * m_width;
            uint32_t topRight = topLeft + 1;
            uint32_t bottomLeft = topLeft + m_width;
            uint32_t bottomRight = bottomLeft + 1;

            // First triangle (top-left, bottom-right, bottom-left)
            m_indices.push_back(topLeft);
            m_indices.push_back(bottomRight);
            m_indices.push_back(bottomLeft);

            // Second triangle (top-left, top-right, bottom-right)
            m_indices.push_back(topLeft);
            m_indices.push_back(topRight);
            m_indices.push_back(bottomRight);
        }
    }

    calculateNormals();
}

void Terrain::calculateNormals()
{

    for (auto& vertex : m_vertices)
    {
        vertex.color = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    // Calculate normals for each triangle
    for (size_t i = 0; i < m_indices.size(); i += 3)
    {
        uint32_t idx0 = m_indices[i];
        uint32_t idx1 = m_indices[i + 1];
        uint32_t idx2 = m_indices[i + 2];

        glm::vec3 v0 = m_vertices[idx0].pos;
        glm::vec3 v1 = m_vertices[idx1].pos;
        glm::vec3 v2 = m_vertices[idx2].pos;

        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 normal = glm::cross(edge1, edge2);

        m_vertices[idx0].color += normal;
        m_vertices[idx1].color += normal;
        m_vertices[idx2].color += normal;
    }

    // Normalize all normals
    for (auto& vertex : m_vertices)
    {
        if (glm::length(vertex.color) > 0.0f)
        {
            vertex.color = glm::normalize(vertex.color);
        }
        else
        {
            vertex.color = glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }
}

float Terrain::barycentric(const glm::vec2& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const
{
    glm::vec2 a2D(a.x, a.z);
    glm::vec2 b2D(b.x, b.z);
    glm::vec2 c2D(c.x, c.z);

    glm::vec2 v0 = b2D - a2D;
    glm::vec2 v1 = c2D - a2D;
    glm::vec2 v2 = p - a2D;

    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;
    if (denom == 0.0f)
        return a.y;

    float v = (d11 * d20 - d01 * d21) / denom;
    float w = (d00 * d21 - d01 * d20) / denom;
    float u = 1.0f - v - w;

    return u * a.y + v * b.y + w * c.y;
}

float Terrain::getHeightAt(float worldX, float worldZ, const glm::vec3& terrainPosition) const
{
    if (m_vertices.empty() || m_heightData.empty())
        return m_heightPlacement;

    // Center the grid using the same offset as generateMesh()
    float offsetX = -m_width * m_gridSpacing / 2.0f;
    float offsetZ = +m_height * m_gridSpacing / 2.0f;


    float localWorldX = worldX - terrainPosition.x;
    float localWorldZ = worldZ - terrainPosition.z;


    float localX = localWorldX - offsetX;
    float localZ = -(localWorldZ - offsetZ);


    int gridX = static_cast<int>(std::floor(localX / m_gridSpacing));
    int gridZ = static_cast<int>(std::floor(localZ / m_gridSpacing));

    // Check bounds
    if (gridX < 0 || gridZ < 0 || gridX >= m_width - 1 || gridZ >= m_height - 1) {

        return m_heightPlacement;
    }

    // Get fractional
    float xCoord = (localX / m_gridSpacing) - gridX;
    float zCoord = (localZ / m_gridSpacing) - gridZ;


    xCoord = glm::clamp(xCoord, 0.0f, 1.0f);
    zCoord = glm::clamp(zCoord, 0.0f, 1.0f);

    glm::vec3 a, b, c;
    int topLeftIndex = gridX + gridZ * m_width;

    // Determine which triangle we're in
    if (xCoord + zCoord <= 1.0f)
    {
        // Upper-left triangle
        a = m_vertices[topLeftIndex].pos;
        b = m_vertices[topLeftIndex + 1].pos;
        c = m_vertices[topLeftIndex + m_width].pos;
    }
    else
    {
        // Lower-right triangle
        a = m_vertices[topLeftIndex + 1 + m_width].pos;
        b = m_vertices[topLeftIndex + m_width].pos;
        c = m_vertices[topLeftIndex + 1].pos;
    }

    float height = barycentric(glm::vec2(localWorldX, localWorldZ), a, b, c);

    return height;
}



glm::vec3 Terrain::getCenter() const
{
    return glm::vec3(0.0f, 0.0f, 0.0f);  // Terrain is centered at origin
}

bool Terrain::loadFromPointCloud(const std::string& filepath, float heightScale, float gridSpacing, float heightPlacement)
{
    m_heightScale = heightScale;
    m_gridSpacing = gridSpacing;
    m_heightPlacement = heightPlacement;

    std::ifstream file(filepath);
    if (!file.is_open())
    {
        std::cerr << "Failed to load point cloud: " << filepath << std::endl;
        return false;
    }

    int totalPoints;
    file >> totalPoints;

    if (totalPoints <= 0)
    {
        std::cerr << "Invalid point count in file: " << totalPoints << std::endl;
        return false;
    }

    std::cout << "Loading " << totalPoints << " points from point cloud..." << std::endl;

    struct Point3D {
        float x, y, z;
    };
    std::vector<Point3D> points;
    points.reserve(totalPoints);

    float x, y, z;
    while (file >> x >> y >> z)
    {
        points.push_back({x, y, z});
    }
    file.close();

    std::cout << "Read " << points.size() << " points" << std::endl;

    // Find bounds
    float minX = points[0].x, maxX = points[0].x;
    float minY = points[0].y, maxY = points[0].y;
    float minZ = points[0].z, maxZ = points[0].z;

    for (const auto& p : points)
    {
        minX = std::min(minX, p.x);
        maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y);
        maxY = std::max(maxY, p.y);
        minZ = std::min(minZ, p.z);
        maxZ = std::max(maxZ, p.z);
    }

    std::cout << "Bounds - X: [" << minX << ", " << maxX << "], "
              << "Y: [" << minY << ", " << maxY << "], "
              << "Z: [" << minZ << ", " << maxZ << "]" << std::endl;

    int gridWidth = static_cast<int>((maxX - minX) / gridSpacing) + 1;
    int gridHeight = static_cast<int>((maxZ - minZ) / gridSpacing) + 1;

    m_width = gridWidth;
    m_height = gridHeight;

    std::cout << "Creating regular grid: " << m_width << "x" << m_height << std::endl;

    std::vector<float> heightMap(m_width * m_height, minY);

    std::cout << "Finding heights for grid cells..." << std::endl;

    int progressStep = m_height / 10;
    if (progressStep == 0) progressStep = 1;

    //Use Avereage of nearby naboInformasjon instead of just nearest
    float searchRadius = gridSpacing * 2.0f;

    for (int d = 0; d < m_height; ++d)
    {
        for (int w = 0; w < m_width; ++w)
        {
            float gridX = minX + w * gridSpacing;
            float gridZ = minZ + d * gridSpacing;

            // Collect all points within search radius
            float heightSum = 0.0f;
            float weightSum = 0.0f;
            int pointsFound = 0;

            for (const auto& p : points)
            {
                float dx = p.x - gridX;
                float dz = p.z - gridZ;
                float dist = std::sqrt(dx * dx + dz * dz);

                if (dist < searchRadius)
                {

                    float weight = 1.0f / (dist + 0.1f);
                    heightSum += p.y * weight;
                    weightSum += weight;
                    pointsFound++;
                }
            }

            if (pointsFound > 0 && weightSum > 0.0f)
            {

                heightMap[w + d * m_width] = heightSum / weightSum;
            }
            else
            {

                float minDist = std::numeric_limits<float>::max();
                float closestHeight = minY;

                for (const auto& p : points)
                {
                    float dx = p.x - gridX;
                    float dz = p.z - gridZ;
                    float dist = dx * dx + dz * dz;

                    if (dist < minDist)
                    {
                        minDist = dist;
                        closestHeight = p.y;
                    }
                }
                heightMap[w + d * m_width] = closestHeight;
            }
        }

        if (d % progressStep == 0)
        {
            std::cout << "  Progress: " << (d * 100 / m_height) << "%" << std::endl;
        }
    }

    std::cout << "Generating mesh with regular triangulation..." << std::endl;
    generateMeshFromHeightMap(heightMap);

    std::cout << "✓ Triangulation done!" << std::endl;
    std::cout << "  Vertices: " << m_vertices.size() << std::endl;
    std::cout << "  Indices: " << m_indices.size() << std::endl;
    std::cout << "  Triangles: " << (m_indices.size() / 3) << std::endl;

    return true;
}

void Terrain::generateMeshFromHeightMap(const std::vector<float>& heightMap)
{
    m_vertices.clear();
    m_indices.clear();
    m_heightData.clear();

    m_vertices.reserve(m_width * m_height);
    m_heightData.reserve(m_width * m_height);

    float vertexXStart = 0.0f - m_width * m_gridSpacing / 2.0f;
    float vertexZStart = 0.0f + m_height * m_gridSpacing / 2.0f;

    for (int d = 0; d < m_height; ++d)
    {
        for (int w = 0; w < m_width; ++w)
        {
            int index = w + d * m_width;
            float height = heightMap[index] * m_heightScale + m_heightPlacement;

            m_heightData.push_back(height);

            Vertex vertex{};

            vertex.pos = glm::vec3(vertexXStart + (w * m_gridSpacing),height, vertexZStart - (d * m_gridSpacing)  );

            vertex.color = glm::vec3(0.5f, 0.5f, 0.5f);

            vertex.texCoord = glm::vec2(w / static_cast<float>(m_width - 1),d / static_cast<float>(m_height - 1) );

            m_vertices.push_back(vertex);
        }
    }

    // Triangulation: connecting grid with triangles
    m_indices.reserve((m_width - 1) * (m_height - 1) * 6);

    for (int d = 0; d < m_height - 1; ++d)
    {
        for (int w = 0; w < m_width - 1; ++w)
        {
            uint32_t topLeft = w + d * m_width;
            uint32_t topRight = topLeft + 1;
            uint32_t bottomLeft = topLeft + m_width;
            uint32_t bottomRight = bottomLeft + 1;

            // Two triangles per quad
            m_indices.push_back(topLeft);
            m_indices.push_back(bottomRight);
            m_indices.push_back(bottomLeft);

            m_indices.push_back(topLeft);
            m_indices.push_back(topRight);
            m_indices.push_back(bottomRight);
        }
    }

    calculateNormals();
}




























