# slicecore — Minimal STL-to-G-code Slicer (C++)

A geometry-processing pipeline that takes a 3D triangle mesh and
converts it into G-code toolpath instructions, layer by layer.

## Pipeline

1. **Mesh generation** — generates a simple STL cube on disk (`cube.stl`),
   standing in for a customer-supplied part geometry.
2. **STL parsing** — reads the binary STL format back in from disk: 80-byte
   header, triangle count, then per-triangle normal + 3 vertices.
3. **Plane-triangle slicing** — for a series of horizontal z-planes (one per
   layer height), intersects every triangle in the mesh against the plane.
   A triangle contributes a line segment to a layer only if the plane
   actually passes through it; the two crossing points are found by linear
   interpolation along the crossing edges.
4. **Loop stitching** — the per-layer segments come out of step 3 in no
   particular order. They're welded into closed perimeter polylines by
   rounding endpoint coordinates to a fixed precision (to handle floating
   point mismatch between triangles that share an edge) and walking from
   segment to segment until each loop closes.
5. **G-code generation** — each closed loop is traced with `G0`/`G1` moves,
   accumulating a simulated extrusion value (`E`) based on distance
   traveled, the same way a real slicer's toolpath does.

## Scope

**Real geometry processing:**
- Actual plane-triangle intersection math, not hardcoded coordinates.
- Actual binary STL parsing (round-trips through a real file on disk).
- Actual path stitching from an unordered set of segments into loops —
  this is the same problem as contour tracing / boundary extraction in any
  geometry pipeline.

**Deliberately out of scope**
- **Infill** — only the outer perimeter is generated; no interior fill
  pattern.
- **Support generation** — no overhang detection or support structures.
- **Arc fitting / path optimization** — toolpaths are raw point-to-point
  linear moves, no smoothing or travel-time optimization.
- **Multi-loop layers** — the stitching logic handles multiple disjoint
  loops per layer (e.g., a mesh with a hole would produce two loops at
  that height), but this hasn't been stress-tested against complex
  multi-body or non-manifold meshes.
- **ASCII STL** — only binary STL is parsed; ASCII STL support would be a
  straightforward addition (text parsing instead of binary reads).

## Build & run

```
g++ -O2 -std=c++17 main.cpp -o slicecore
./slicecore
```

This generates `cube.stl` (the test part) and `output.gcode` (the sliced
toolpath) in the current directory. The program prints a summary: layer
count, total segments found, total loops stitched, and total simulated
extrusion.

**Sanity check built into the run**: for a solid cube, every layer should
produce exactly one closed loop. The program prints "expected N for a solid
cube" and this should match the actual stitched loop count — if the
stitching logic were broken (e.g., loops not closing, segments dropped),
this number would diverge from the layer count.

## Trying it on a different shape

`generateCube(size)` can be swapped for any other triangle mesh — including
a real STL file exported from a CAD tool, by pointing `readBinarySTL` at it
directly instead of generating and round-tripping the cube. Non-cube shapes
will produce non-square loops but the same pipeline handles them, since
none of the slicing/stitching logic assumes a cube.

## Future Plans

- Add ASCII STL parsing.
- Add a basic infill pattern (e.g., a zigzag or grid) inside each perimeter
  loop, not just the outer boundary.
- Handle multi-loop layers robustly (meshes with holes, e.g., a slice
  through a torus or a mesh with a cavity).
- Add overhang detection: compare each layer's footprint to the layer below
  and flag regions with no support underneath.
- Replace linear point-to-point G-code moves with arc fitting for curved
  perimeters.
