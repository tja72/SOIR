#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <algorithm>

// A simple structure to hold a 3D point.
struct Point {
    float x, y, z;
};

// A simple mesh structure: a list of vertices and a list of faces.
// Each face is represented by a vector of vertex indices.
struct ReductSimpleMesh {
    std::vector<Point> vertices;
    std::vector<std::vector<int>> faces;
};

// A helper structure to represent a grid cell index.
struct GridCell {
    int ix, iy, iz;
    bool operator==(const GridCell& other) const {
        return ix == other.ix && iy == other.iy && iz == other.iz;
    }
};

// A hash function for GridCell so it can be used as a key in unordered_map.
struct GridCellHash {
    std::size_t operator()(const GridCell& cell) const {
        std::size_t h1 = std::hash<int>()(cell.ix);
        std::size_t h2 = std::hash<int>()(cell.iy);
        std::size_t h3 = std::hash<int>()(cell.iz);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

//
// Function to load an OFF file into a SimpleMesh structure.
//
bool loadOFF(const std::string& filename, ReductSimpleMesh& mesh) {
    std::ifstream in(filename);
    if (!in) {
        std::cerr << "Error: Could not open file " << filename << "\n";
        return false;
    }

    //std::string header;
    //std::getline(in, header);
    //if (header != "OFF") {
    //    std::cerr << "Error: File " << filename << " is not a valid OFF file (missing 'OFF' header).\n";
    //    return false;
    //}

    // Read counts: number of vertices, faces, and edges (edges are ignored).
    int numVertices = 0, numFaces = 0, numEdges = 0;
    while (in.good()) {
        std::string line;
        std::getline(in, line);
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream iss(line);
        if (!(iss >> numVertices >> numFaces >> numEdges)) {
            std::cerr << "Error: Could not read vertex/face counts from OFF file.\n";
            return false;
        }
        break;
    }

    // Read vertices.
    mesh.vertices.resize(numVertices);
    for (int i = 0; i < numVertices; ++i) {
        in >> mesh.vertices[i].x >> mesh.vertices[i].y >> mesh.vertices[i].z;
    }

    // Read faces.
    mesh.faces.resize(numFaces);
    for (int i = 0; i < numFaces; ++i) {
        int verticesInFace = 0;
        in >> verticesInFace;
        if (verticesInFace < 3) { // A valid face must have at least 3 vertices.
            std::cerr << "Warning: Face " << i << " has fewer than 3 vertices; skipping.\n";
            mesh.faces[i].clear();
            continue;
        }
        mesh.faces[i].resize(verticesInFace);
        for (int j = 0; j < verticesInFace; ++j) {
            in >> mesh.faces[i][j];
        }
    }

    return true;
}

//
// save ReductMesh to an OFF file.
//
bool writeOFF(const std::string& filename, const ReductSimpleMesh& mesh) {
    std::ofstream out(filename);
    if (!out) {
        std::cerr << "Error: Could not open file " << filename << " for writing.\n";
        return false;
    }

    out << "OFF\n";
    out << mesh.vertices.size() << " " << mesh.faces.size() << " 0\n";
    for (const auto& v : mesh.vertices) {
        out << v.x << " " << v.y << " " << v.z << "\n";
    }
    for (const auto& face : mesh.faces) {
        if (face.empty())
            continue;
        out << face.size();
        for (int idx : face) {
            out << " " << idx;
        }
        out << "\n";
    }

    return true;
}

//
// Function to reduce the resolution of a mesh using vertex clustering (grid decimation).
//
// Parameters:
//   mesh             - The input high-resolution mesh.
//   numCellsPerAxis  - The number of grid cells per axis. A smaller number means more aggressive decimation.
//
// Returns ReductMesh with fewer vertices and updated faces.
//
ReductSimpleMesh reduceMesh(const ReductSimpleMesh& mesh, int numCellsPerAxis) {
    ReductSimpleMesh newMesh;
    if (mesh.vertices.empty())
        return newMesh;

    // Compute the axis-aligned bounding box of the mesh.
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();

    for (const auto& v : mesh.vertices) {
        minX = std::min(minX, v.x);
        minY = std::min(minY, v.y);
        minZ = std::min(minZ, v.z);
        maxX = std::max(maxX, v.x);
        maxY = std::max(maxY, v.y);
        maxZ = std::max(maxZ, v.z);
    }

    // Compute the cell size for each axis.
    float dx = (maxX - minX) / numCellsPerAxis;
    float dy = (maxY - minY) / numCellsPerAxis;
    float dz = (maxZ - minZ) / numCellsPerAxis;
    if (dx == 0) dx = 1.0f;
    if (dy == 0) dy = 1.0f;
    if (dz == 0) dz = 1.0f;

    // Map each vertex to a grid cell.
    std::unordered_map<GridCell, std::vector<int>, GridCellHash> cellVertices;
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const auto& v = mesh.vertices[i];
        GridCell cell;
        cell.ix = static_cast<int>(std::floor((v.x - minX) / dx));
        cell.iy = static_cast<int>(std::floor((v.y - minY) / dy));
        cell.iz = static_cast<int>(std::floor((v.z - minZ) / dz));
        // Clamp indices to ensure they lie within [0, numCellsPerAxis-1]
        cell.ix = std::min(cell.ix, numCellsPerAxis - 1);
        cell.iy = std::min(cell.iy, numCellsPerAxis - 1);
        cell.iz = std::min(cell.iz, numCellsPerAxis - 1);
        cellVertices[cell].push_back(static_cast<int>(i));
    }

    // For each cell, compute a representative vertex (the average of all vertices in the cell)
    // and build a mapping from original vertex index to the new vertex index.
    std::unordered_map<int, int> vertexMapping; // key: old vertex index, value: new vertex index
    for (const auto& pair : cellVertices) {
        const std::vector<int>& indices = pair.second;
        Point avg = { 0.0f, 0.0f, 0.0f };
        for (int idx : indices) {
            avg.x += mesh.vertices[idx].x;
            avg.y += mesh.vertices[idx].y;
            avg.z += mesh.vertices[idx].z;
        }
        avg.x /= indices.size();
        avg.y /= indices.size();
        avg.z /= indices.size();

        int newIndex = static_cast<int>(newMesh.vertices.size());
        newMesh.vertices.push_back(avg);
        for (int idx : indices) {
            vertexMapping[idx] = newIndex;
        }
    }

    // Rebuild faces: for each face in the original mesh, map its vertices to the new vertex indices.
    // If the mapped face has fewer than 3 unique vertices, skip it.
    for (const auto& face : mesh.faces) {
        std::vector<int> newFace;
        for (int idx : face) {
            // Map the original vertex to its new vertex
            int mapped = vertexMapping[idx];
            newFace.push_back(mapped);
        }
        // Remove duplicate indices.
        std::sort(newFace.begin(), newFace.end());
        newFace.erase(std::unique(newFace.begin(), newFace.end()), newFace.end());
        if (newFace.size() < 3)
            continue; // Skip degenerate faces.
        newMesh.faces.push_back(newFace);
    }

    return newMesh;
}

//
// Main function: reads an OFF file, reduces the mesh resolution, and writes the result.
//
int reduce_main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " input.off output.off numCellsPerAxis\n";
        std::cerr << "  numCellsPerAxis: grid resolution per axis (e.g., 10 means 10x10x10 cells).\n";
        return 1;
    }

    std::string inputFilename = argv[1];
    std::string outputFilename = argv[2];
    int numCellsPerAxis = std::stoi(argv[3]);
    if (numCellsPerAxis < 1) {
        std::cerr << "Error: numCellsPerAxis must be at least 1.\n";
        return 1;
    }

    ReductSimpleMesh mesh;
    if (!loadOFF(inputFilename, mesh)) {
        std::cerr << "Error: Failed to load the input mesh.\n";
        return 1;
    }

    // Reduce the mesh resolution.
    ReductSimpleMesh reducedMesh = reduceMesh(mesh, numCellsPerAxis);

    if (!writeOFF(outputFilename, reducedMesh)) {
        std::cerr << "Error: Failed to write the reduced mesh.\n";
        return 1;
    }

    std::cout << "Successfully reduced the mesh and saved to " << outputFilename << "\n";
    return 0;
}
