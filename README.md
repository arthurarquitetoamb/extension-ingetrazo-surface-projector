# Surface Contour Projector and Extruder
**IngeTrazo Plugin**

Project cutter face(s) onto a target surface along a global axis, then extrude the result into solid contours.  
Ideal for embankment slopes, sidewalks, curbs, surface markings, and surfaces with holes for a model.

<img width="595" height="373" alt="Face Grupo README" src="https://github.com/user-attachments/assets/e4024728-49ac-4cdb-a4be-d598c8d71360" />

---

## Overview

The plugin performs a **first-hit raycast** from selected cutter faces onto a target group.  
It builds the intercepted contour as a 3D mesh, with optional extrusion along the projection axis or the hit surface normal.  
Independent base and cap scaling let you create tapered profiles, truncated pyramids, planified tops, and inclined embankments without manual cleanup.

---

## Key Features

| Feature | What it enables |
|---------|-----------------|
| **Build embankment slopes** | Extrude a base contour up/down a terrain surface to form retaining walls or sloped embankments. |
| **Lay sidewalks and curbs** | Project sidewalk outlines onto irregular ground and extrude them as curbs or walkway slabs. |
| **Modeling productivity** | One-click projection + extrusion replaces manual vertex snapping and face bridging. |
| **Flexible extrusion for inclined profiles** | Extrude along the projection axis or along the target face normal to follow complex terrain. |
| **Independent base and cap scales** | Shrink or expand the base and cap separately to create truncated pyramids or tapered slabs. |
| **Generate markings on complex surfaces** | Project closed contours onto uneven terrain to create painted areas, tactile paving, or signage bases. |
| **Work with hollow projections** | Holes in cutter faces are preserved: no faces are generated inside voids, and exterior walls follow the real contour. |

---

## Installation

1. Copy `Surface Contour Projector and Extruder.py` to:
2. Open **Extensions > Surface Contour Projector and Extruder…**

---

## How to Use

1. **Select cutter face(s)** in **Sub-Object (Face)** mode.  
You may select multiple faces at different heights, including faces with holes.
2. **Select exactly 1 Target Group** (the terrain, slab, or surface to project onto).
3. Open the plugin panel and choose:
- **Axis** (X, Y or Z) — global projection direction.
- **Extrude Along** — *Projection Axis* or *Face Normal* (follows the target surface).
- **Extrusion Distance** — positive for relief, negative for recess. Zero creates a single projected face.
- **Scale Factor (Pre)** — scales the cutter footprint before projection.
- **Scale Base After Extrusion** — scales only the base after extrusion (great for tapered footprints).
- **Scale Cap Along Axis** — flattens or exaggerates only the top face along the projection axis.
4. Click **Project and Extrude**.
5. The resulting group is created and selected automatically.

---

## Controls

| Control | Description |
|---------|-------------|
| **Units** | Meters, Centimeters, Millimeters, Inches, Feet. Affects Lift and Extrusion Distance spin boxes. |
| **Axis** | Global projection axis (X / Y / Z). Ray direction is inferred automatically from the cutter normal. |
| **Extrude Along** | *Projection Axis*: straight extrusion along X/Y/Z.<br>*Face Normal*: extrusion follows the normal of the target face hit by the ray. |
| **Lift (Offset)** | Small offset applied after the ray hit to avoid z-fighting with the target surface. |
| **Extrusion Distance** | Thickness of the extrusion. Positive = outward/upward, Negative = inward/downward. Zero = single face only. |
| **Scale Factor (Pre)** | Scales the cutter polygon around its centroid before the raycast. `1.0` = no change. |
| **Scale Base After Extrusion** | When enabled, applies a final scale only to the base face after extrusion. The cap keeps the pre-scale, forming a truncated pyramid. |
| **Post-extrusion Scale (Base)** | Factor applied only to the base after extrusion. `1.0` = no change. |
| **Scale Cap Along Axis** | When enabled, scales only the last extrusion face (cap) along the projection axis. `0` planifies (flattens) the cap; `>1` exaggerates the slope. |
| **Cap Axis Scale** | Factor applied only to the cap along the chosen axis. |
| **Project and Extrude** | Executes the operation with the current settings. |

---

## Typical Workflows

### Embankment Slopes
1. Draw the embankment footprint as a closed face.
2. Project onto the terrain along Z.
3. Enable **Extrude Along: Face Normal** and set a positive **Extrusion Distance**.
4. Enable **Scale Base After Extrusion** with a factor < 1.0 to taper the footprint upward.
5. Optionally enable **Scale Cap Along Axis** with `0` to create a flat top at a fixed elevation.

### Sidewalks and Curbs on Terrain
1. Draw the sidewalk outline (with curb radius if needed) as one or more faces.
2. Project onto the terrain along Z.
3. Set **Extrude Along: Projection Axis** and a small negative/positive **Extrusion Distance** to form the curb thickness.
4. Use **Scale Factor (Pre)** to fine-tune the sidewalk width before projection.

### Surface Markings on Complex Surfaces
1. Draw the marking contour (tactile paving, parking symbol, etc.).
2. Project with **Extrusion Distance = 0** to generate a single face that conforms exactly to the target surface.
3. Use **Lift (Offset)** to raise the marking slightly above the surface for visibility.

### Hollow / Void Projections
1. Select a cutter face that contains interior holes (e.g., a donut-shaped footprint).
2. The plugin triangulates only the solid material; holes remain empty.
3. Exterior walls are built around the outer contour only, preserving the void.

---

## Notes

- **Ray direction** is inferred automatically from the average cutter normal on the projection axis. If no hit is found, the opposite direction is tried automatically.
- If a large percentage of cutter sample points miss the target, a warning dialog appears. This usually means the cutter is larger than the target surface and may produce floating or disconnected geometry.
- All generated edges are hidden by default; only faces are visible.
- Multiple cutter faces at different heights are supported in a single operation.

---

## Requirements

- IngeTrazo with Python/PySide6 plugin support.
- `shapely` installed in the environment.
- `core.history`, `core.group`, `core.mesh` available from the host application.

---

## License

This project is licensed under the **GNU General Public License v3.0 or later (GPL-3.0-or-later)**. See the [LICENSE](LICENSE) file for details.

## Tests

This plugin has been tested on **IngeTrazo 0.6.3.2**.

*Surface Contour Projector and Extruder — IngeTrazo Plugin*  
*Use at your own risk. Always backup your scene before running batch operations.*
