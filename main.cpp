// main.cpp — slicecore: a minimal STL-to-G-code slicer
//
// Pipeline: load a triangle mesh from a binary STL file -> slice it into
// horizontal layers by intersecting the mesh with a series of z-planes ->
// stitch the resulting edge segments into closed perimeter loops per layer
// -> emit G-code toolpath instructions that trace each loop.
//
// This intentionally does the geometry-processing core of what a real
// slicer (Cura, PrusaSlicer, etc.) does, without the parts that would
// balloon scope: no infill patterns, no support generation, no multi-part
// packing, no arc fitting. Perimeter-only, single-loop-per-triangle-set
// slicing is still real geometry processing: plane-triangle intersection,
// vertex welding, and path stitching are the same primitives a production
// slicer builds on top of.
//
// Build:   g++ -O2 -std=c++17 main.cpp -o slicecore
// Run:     ./slicecore
//
// The program generates a small STL cube on disk (cube.stl), reads it back
// in, slices it, and writes layer-by-layer G-code to output.gcode.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Geometry primitives
// ---------------------------------------------------------------------------
struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
};

struct Triangle {
    Vec3 normal;
    Vec3 v0, v1, v2;
};

// A single 2D point within a layer (slicing is done per z-plane, so we only
// need x/y once the layer height is fixed).
struct Point2 {
    float x, y;
};

// ---------------------------------------------------------------------------
// STL I/O (binary format only — the common case for real STL exports)
//
// Binary STL layout:
//   80 bytes  : header (ignored)
//   4 bytes   : uint32 triangle count
//   per triangle (50 bytes):
//     12 bytes : normal (3x float32)
//     36 bytes : v0, v1, v2 (3x float32 each)
//     2 bytes  : attribute byte count (ignored)
// ---------------------------------------------------------------------------
static Vec3 readVec3(std::ifstream& in) {
    float x, y, z;
    in.read(reinterpret_cast<char*>(&x), 4);
    in.read(reinterpret_cast<char*>(&y), 4);
    in.read(reinterpret_cast<char*>(&z), 4);
    return {x, y, z};
}

static void writeVec3(std::ofstream& out, const Vec3& v) {
    out.write(reinterpret_cast<const char*>(&v.x), 4);
    out.write(reinterpret_cast<const char*>(&v.y), 4);
    out.write(reinterpret_cast<const char*>(&v.z), 4);
}

bool writeBinarySTL(const std::string& path, const std::vector<Triangle>& tris) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;

    char header[80] = {0};
    std::string tag = "slicecore-generated";
    std::memcpy(header, tag.c_str(), std::min(tag.size(), sizeof(header)));
    out.write(header, 80);

    uint32_t count = static_cast<uint32_t>(tris.size());
    out.write(reinterpret_cast<const char*>(&count), 4);

    uint16_t attr = 0;
    for (const auto& t : tris) {
        writeVec3(out, t.normal);
        writeVec3(out, t.v0);
        writeVec3(out, t.v1);
        writeVec3(out, t.v2);
        out.write(reinterpret_cast<const char*>(&attr), 2);
    }
    return true;
}

bool readBinarySTL(const std::string& path, std::vector<Triangle>& tris) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;

    char header[80];
    in.read(header, 80);

    uint32_t count;
    in.read(reinterpret_cast<char*>(&count), 4);

    tris.clear();
    tris.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        Triangle t;
        t.normal = readVec3(in);
        t.v0 = readVec3(in);
        t.v1 = readVec3(in);
        t.v2 = readVec3(in);
        uint16_t attr;
        in.read(reinterpret_cast<char*>(&attr), 2);
        tris.push_back(t);
    }
    return true;
}

// ---------------------------------------------------------------------------
// Test mesh generation: a simple axis-aligned cube, two triangles per face.
// Stands in for "a customer part geometry" so the pipeline can be exercised
// end-to-end without needing an external STL file.
// ---------------------------------------------------------------------------
std::vector<Triangle> generateCube(float size) {
    std::vector<Triangle> tris;

    auto addQuad = [&](Vec3 a, Vec3 b, Vec3 c, Vec3 d, Vec3 n) {
        tris.push_back({n, a, b, c});
        tris.push_back({n, a, c, d});
    };

    Vec3 p000{0, 0, 0}, p100{size, 0, 0}, p110{size, size, 0}, p010{0, size, 0};
    Vec3 p001{0, 0, size}, p101{size, 0, size}, p111{size, size, size}, p011{0, size, size};

    addQuad(p000, p100, p110, p010, {0, 0, -1});  // bottom
    addQuad(p001, p011, p111, p101, {0, 0, 1});   // top
    addQuad(p000, p001, p101, p100, {0, -1, 0});  // front
    addQuad(p010, p110, p111, p011, {0, 1, 0});   // back
    addQuad(p000, p010, p011, p001, {-1, 0, 0});  // left
    addQuad(p100, p101, p111, p110, {1, 0, 0});   // right

    return tris;
}

// ---------------------------------------------------------------------------
// Slicing: intersect the mesh with a horizontal plane at height z.
//
// For each triangle, classify its three vertices by signed distance to the
// plane (d = vertex.z - planeZ). A triangle contributes a segment to this
// layer only if the plane passes through it (not all vertices on the same
// side). For each edge whose endpoints have opposite-sign distances, linear
// interpolation gives the exact crossing point. A triangle crossed by the
// plane contributes exactly one segment (two crossing points), except for
// degenerate cases (a vertex exactly on the plane, or a whole face lying in
// the plane) which we skip — acceptable for a minimal slicer.
// ---------------------------------------------------------------------------
struct Segment {
    Point2 a, b;
};

static bool intersectEdge(const Vec3& p1, const Vec3& p2, float planeZ, Point2& out) {
    float d1 = p1.z - planeZ;
    float d2 = p2.z - planeZ;
    if ((d1 > 0) == (d2 > 0)) return false;  // both same side, no crossing
    float t = d1 / (d1 - d2);
    Vec3 p = p1 + (p2 - p1) * t;
    out = {p.x, p.y};
    return true;
}

std::vector<Segment> sliceLayer(const std::vector<Triangle>& tris, float planeZ) {
    std::vector<Segment> segments;

    for (const auto& t : tris) {
        std::vector<Point2> crossings;
        Point2 pt;
        if (intersectEdge(t.v0, t.v1, planeZ, pt)) crossings.push_back(pt);
        if (intersectEdge(t.v1, t.v2, planeZ, pt)) crossings.push_back(pt);
        if (intersectEdge(t.v2, t.v0, planeZ, pt)) crossings.push_back(pt);

        // A plane crossing a triangle (non-degenerate case) hits exactly
        // two edges, producing one segment.
        if (crossings.size() == 2) {
            segments.push_back({crossings[0], crossings[1]});
        }
    }

    return segments;
}

// ---------------------------------------------------------------------------
// Loop stitching: chain unordered segments into closed perimeter polylines.
//
// Real meshes produce floating point coordinates that rarely match exactly
// even when they're meant to be the same point (shared triangle edges), so
// endpoints are "welded" by rounding to a fixed precision before using them
// as map keys. Each welded point is linked to the segments touching it;
// starting from an arbitrary unused segment, we walk from endpoint to
// endpoint until the loop closes back on itself.
// ---------------------------------------------------------------------------
static int64_t weldKey(const Point2& p) {
    // Round to 3 decimal places (micron precision at mm scale) and pack
    // into a single integer key.
    int32_t xi = static_cast<int32_t>(std::round(p.x * 1000.0f));
    int32_t yi = static_cast<int32_t>(std::round(p.y * 1000.0f));
    return (static_cast<int64_t>(xi) << 32) ^ static_cast<uint32_t>(yi);
}

std::vector<std::vector<Point2>> stitchLoops(const std::vector<Segment>& segments) {
    std::multimap<int64_t, size_t> pointToSegment;  // welded point -> segment index
    std::vector<bool> used(segments.size(), false);

    for (size_t i = 0; i < segments.size(); ++i) {
        pointToSegment.emplace(weldKey(segments[i].a), i);
        pointToSegment.emplace(weldKey(segments[i].b), i);
    }

    std::vector<std::vector<Point2>> loops;

    for (size_t start = 0; start < segments.size(); ++start) {
        if (used[start]) continue;

        std::vector<Point2> loop;
        size_t current = start;
        used[current] = true;
        loop.push_back(segments[current].a);
        Point2 head = segments[current].b;
        loop.push_back(head);

        int64_t startKey = weldKey(segments[start].a);

        // Walk forward until we get back to the loop's starting point.
        for (int guard = 0; guard < static_cast<int>(segments.size()) + 1; ++guard) {
            if (weldKey(head) == startKey) break;  // loop closed

            auto range = pointToSegment.equal_range(weldKey(head));
            size_t next = segments.size();
            for (auto it = range.first; it != range.second; ++it) {
                if (!used[it->second]) {
                    next = it->second;
                    break;
                }
            }
            if (next == segments.size()) break;  // dead end (open mesh / gap)

            used[next] = true;
            // Continue from whichever endpoint of `next` isn't the shared one.
            Point2 nextHead = (weldKey(segments[next].a) == weldKey(head))
                                   ? segments[next].b
                                   : segments[next].a;
            loop.push_back(nextHead);
            head = nextHead;
            current = next;
        }

        if (loop.size() >= 3) {
            loops.push_back(std::move(loop));
        }
    }

    return loops;
}

// ---------------------------------------------------------------------------
// G-code generation: trace each loop with linear moves, simulating
// extrusion accumulation based on distance traveled.
// ---------------------------------------------------------------------------
struct GcodeState {
    float extrusion = 0.0f;
    // mm of filament extruded per mm of travel, for a nominal 0.4mm nozzle /
    // 0.2mm layer height — a stand-in constant, not a calibrated value.
    static constexpr float EXTRUSION_PER_MM = 0.033f;
};

void writeGcode(std::ofstream& out, const std::vector<std::vector<Point2>>& loops,
                 float layerZ, int layerIndex, GcodeState& state) {
    out << "; layer " << layerIndex << " at z=" << std::fixed << std::setprecision(3)
        << layerZ << "\n";

    for (const auto& loop : loops) {
        if (loop.empty()) continue;

        // Rapid travel move to the loop's start point at this layer height.
        out << "G0 F3000 X" << std::fixed << std::setprecision(3) << loop[0].x << " Y"
            << loop[0].y << " Z" << layerZ << "\n";

        for (size_t i = 1; i < loop.size(); ++i) {
            float dx = loop[i].x - loop[i - 1].x;
            float dy = loop[i].y - loop[i - 1].y;
            float dist = std::sqrt(dx * dx + dy * dy);
            state.extrusion += dist * GcodeState::EXTRUSION_PER_MM;

            out << "G1 F1200 X" << loop[i].x << " Y" << loop[i].y << " E" << std::fixed
                << std::setprecision(4) << state.extrusion << "\n";
        }
    }
}

// ---------------------------------------------------------------------------
// Main pipeline
// ---------------------------------------------------------------------------
int main() {
    const float cubeSize = 20.0f;      // mm
    const float layerHeight = 0.2f;    // mm

    std::cout << "slicecore: STL -> G-code pipeline\n";
    std::cout << "===================================\n\n";

    // 1. Generate a test mesh and write it to disk, so we have a real STL
    //    file to read back in (stands in for a customer-supplied part).
    std::vector<Triangle> mesh = generateCube(cubeSize);
    if (!writeBinarySTL("cube.stl", mesh)) {
        std::cerr << "Failed to write cube.stl\n";
        return 1;
    }
    std::cout << "Generated cube.stl: " << mesh.size() << " triangles, "
              << cubeSize << "mm cube\n";

    // 2. Read the mesh back in from disk, exercising the real STL parser.
    std::vector<Triangle> loaded;
    if (!readBinarySTL("cube.stl", loaded)) {
        std::cerr << "Failed to read cube.stl\n";
        return 1;
    }
    std::cout << "Loaded " << loaded.size() << " triangles from cube.stl\n\n";

    // 3. Slice into layers and stitch perimeter loops, writing G-code as we go.
    int numLayers = static_cast<int>(cubeSize / layerHeight);
    std::ofstream gcode("output.gcode");
    gcode << "; slicecore generated toolpath\n";
    gcode << "G21 ; units = mm\n";
    gcode << "G90 ; absolute positioning\n";
    gcode << "M104 S200 ; set nozzle temp (nominal)\n";
    gcode << "G28 ; home all axes\n\n";

    GcodeState state;
    int totalSegments = 0;
    int totalLoops = 0;

    for (int layer = 1; layer < numLayers; ++layer) {  // skip layer 0 (exact z=0 is degenerate)
        float z = layer * layerHeight;
        auto segments = sliceLayer(loaded, z);
        auto loops = stitchLoops(segments);

        totalSegments += static_cast<int>(segments.size());
        totalLoops += static_cast<int>(loops.size());

        writeGcode(gcode, loops, z, layer, state);
    }

    gcode << "\nM104 S0 ; nozzle off\n";
    gcode << "G28 ; return home\n";
    gcode.close();

    std::cout << "Sliced " << (numLayers - 1) << " layers\n";
    std::cout << "Total edge segments: " << totalSegments << "\n";
    std::cout << "Total stitched loops: " << totalLoops << " (expected "
              << (numLayers - 1) << " for a solid cube — one loop per layer)\n";
    std::cout << "Total simulated extrusion: " << std::fixed << std::setprecision(2)
              << state.extrusion << " mm of filament\n\n";
    std::cout << "Wrote output.gcode\n";

    return 0;
}
