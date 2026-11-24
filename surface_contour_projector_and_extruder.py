"""Surface Contour Projector and Extruder — IngeTrazo Plugin.

Project cutter face(s) onto a target surface along a global axis, then extrude
the intercepted contour into solid contours. Ideal for embankment slopes,
sidewalks, curbs, surface markings, and hollow terrain features.

Install: %APPDATA%\\ingetrazo\\plugins\\surface_contour_projector.py
Restart IngeTrazo -> Extensions > Surface Contour Projector and Extruder…
"""

from __future__ import annotations

import sys
import math
import traceback
from dataclasses import dataclass
from typing import List, Tuple, Optional

# --- Dependencies ---
try:
    from shapely.geometry import Polygon, MultiPolygon, LinearRing
    from shapely.ops import unary_union, triangulate
    from shapely.affinity import scale as shapely_scale
    SHAPELY_OK = True
except ImportError as e:
    SHAPELY_OK = False; SHAPELY_ERR = e

try:
    from core.history import SnapshotImport
    from core.group import iter_placements, Group
    from core.mesh import Mesh
    CORE_OK = True
except ImportError as e:
    CORE_OK = False; CORE_ERR = e

from PySide6.QtWidgets import (QWidget, QVBoxLayout, QFormLayout, QComboBox, QDoubleSpinBox,
                               QPushButton, QLabel, QHBoxLayout, QMessageBox, QGroupBox,
                               QCheckBox)
from PySide6.QtCore import Qt
from PySide6.QtGui import QVector3D


PLUGIN_NAME = "Surface Contour Projector and Extruder"
DEFAULT_LIFT = 0.01
DEFAULT_EXTRUSION = 0.0
DEFAULT_SCALE = 1.0
DEFAULT_EXTRA_SCALE = 1.0
DEFAULT_CAP_AXIS_SCALE = 1.0
RAY_EPS = 1e-6
RAY_FAR = 1e6
MISS_WARN_RATIO = 0.15   # warn if >15% of sample points miss target

AXES = ["X", "Y", "Z"]
EXTRUDE_ALONG = ["Projection Axis", "Face Normal"]

AXIS_IDX = {"X": 0, "Y": 1, "Z": 2}


# -----------------------------------------------------------------------------
# Unit System
# -----------------------------------------------------------------------------
class UnitSystem:
    UNITS = {
        "Meters":        {"suffix": " m", "decimals": 3, "step": 0.001, "factor": 1.0},
        "Centimeters":   {"suffix": " cm", "decimals": 2, "step": 0.01,  "factor": 0.01},
        "Millimeters":   {"suffix": " mm", "decimals": 1, "step": 0.1,   "factor": 0.001},
        "Inches":        {"suffix": " in", "decimals": 3, "step": 0.001, "factor": 0.0254},
        "Feet":          {"suffix": " ft", "decimals": 3, "step": 0.001, "factor": 0.3048},
    }
    DEFAULT_KEY = "Meters"
    def __init__(self, app=None):
        self.current_key = self.DEFAULT_KEY; self._try_auto_detect(app)
    def _try_auto_detect(self, app):
        candidates = []
        if app:
            candidates.append(("app.preferences.unit", lambda: getattr(getattr(app, "preferences", None), "unit", None)))
            candidates.append(("app.unit", lambda: getattr(app, "unit", None)))
            candidates.append(("scene.unit", lambda: getattr(getattr(app, "scene", None), "unit", None)))
        for name, getter in candidates:
            try:
                val = getter()
                if val:
                    val_str = str(val).strip().lower()
                    mapping = {"m":"Meters","meter":"Meters","meters":"Meters","cm":"Centimeters",
                               "centimeter":"Centimeters","mm":"Millimeters","millimeter":"Millimeters",
                               "in":"Inches","inch":"Inches","inches":"Inches","ft":"Feet","foot":"Feet","feet":"Feet"}
                    if val_str in mapping: self.current_key = mapping[val_str]; print(f"[SurfaceContourProjector] Units: {self.current_key}"); return
            except: pass
        print(f"[SurfaceContourProjector] Units default: {self.current_key}")
    def get_props(self) -> dict: return self.UNITS.get(self.current_key, self.UNITS[self.DEFAULT_KEY])
    def set_unit(self, key: str):
        if key in self.UNITS: self.current_key = key


def dlog(msg: str):
    try: print(f"[SurfaceContourProjector] {msg}"); sys.stdout.flush()
    except: pass

def ulog(viewport, title: str, msg: str, icon=QMessageBox.Icon.Information):
    dlog(f"UI_LOG [{title}]: {msg}")
    try: QMessageBox.message(viewport.window(), title, msg, QMessageBox.StandardButton.Ok, icon)
    except: pass


# -----------------------------------------------------------------------------
# Math Core
# -----------------------------------------------------------------------------
@dataclass(frozen=True)
class Vec3:
    x: float; y: float; z: float
    def __getitem__(self, i): return (self.x, self.y, self.z)[i]
    def __sub__(self, o): return Vec3(self.x-o.x, self.y-o.y, self.z-o.z)
    def __add__(self, o): return Vec3(self.x+o.x, self.y+o.y, self.z+o.z)
    def __mul__(self, s): return Vec3(self.x*s, self.y*s, self.z*s)
    def __rmul__(self, s): return self * s
    def dot(self, o): return self.x*o.x + self.y*o.y + self.z*o.z
    def cross(self, o): return Vec3(self.y*o.z-self.z*o.y, self.z*o.x-self.x*o.z, self.x*o.y-self.y*o.x)
    def length(self): return math.sqrt(self.dot(self))
    def normalized(self): l=self.length(); return Vec3(0,0,0) if l<1e-12 else self*(1.0/l)
    def to_qv(self): return QVector3D(self.x, self.y, self.z)
    @staticmethod
    def from_qv(v): return Vec3(v.x(), v.y(), v.z())

def qv_to_vec3(qv) -> Vec3: return Vec3(qv.x(), qv.y(), qv.z())

@dataclass
class Tri3D:
    v0: Vec3; v1: Vec3; v2: Vec3; edge1: Vec3; edge2: Vec3; normal: Vec3
    u_idx: int; v_idx: int; w_idx: int
    @staticmethod
    def from_points(p0, p1, p2, u_idx, v_idx, w_idx) -> "Tri3D":
        e1=p1-p0; e2=p2-p0; n=e1.cross(e2)
        return Tri3D(p0,p1,p2,e1,e2,n,u_idx,v_idx,w_idx)
    def intersect_ray(self, orig: Vec3, dir: Vec3) -> Optional[float]:
        h = dir.cross(self.edge2); a = self.edge1.dot(h)
        if a > -RAY_EPS: return None
        f = 1.0/a; s = orig - self.v0; u = f * s.dot(h)
        if u < 0.0 or u > 1.0: return None
        q = s.cross(self.edge1); v = f * dir.dot(q)
        if v < 0.0 or u+v > 1.0: return None
        t = f * self.edge2.dot(q)
        return t if t > RAY_EPS else None


# -----------------------------------------------------------------------------
# Scene Helpers
# -----------------------------------------------------------------------------
def _map_point(pt, mx) -> Optional[Vec3]:
    try: return qv_to_vec3(pt) if mx is None else qv_to_vec3(mx.map(pt))
    except: return None

def _iter_selection(scene) -> Tuple[List[Tuple[object, object, object]], List[object]]:
    sel = list(getattr(scene, "selection", None) or ())
    groups = [o for o in sel if hasattr(o, "mesh") and hasattr(o, "name")]
    loose = [o for o in sel if o not in groups]
    proj_faces=[]; seen=set()
    for item in loose:
        if hasattr(item, "loop") and hasattr(item, "normal"):
            key=(id(item), id(getattr(item,"mesh",None)))
            if key not in seen: seen.add(key); proj_faces.append((item, None, getattr(item,"mesh",None)))
    return proj_faces, groups

def _get_cutter_polygon_2d(face, mx, u_idx, v_idx) -> Optional[Polygon]:
    try:
        loop_verts=list(face.loop)
        if len(loop_verts)<3: return None
        outer=[]
        for v in loop_verts:
            wp=_map_point(v.position,mx)
            if not wp: return None
            outer.append((wp[u_idx], wp[v_idx]))
        holes=[]
        for hl in getattr(face,"hole_loops",[]) or []:
            hv=[]
            for v in hl:
                wp=_map_point(v.position,mx)
                if not wp: return None
                hv.append((wp[u_idx], wp[v_idx]))
            if len(hv)>=3: holes.append(hv)
        poly=Polygon(outer, holes)
        return poly.buffer(0) if not poly.is_valid else poly
    except: return None

def _collect_target_triangles(group, u_idx, v_idx, w_idx) -> List[Tri3D]:
    tris=[]; meshes=[]
    try:
        for pl, mx in iter_placements(group):
            if getattr(pl,"mesh",None): meshes.append((pl.mesh, mx))
    except:
        if getattr(group,"mesh",None): meshes.append((group.mesh, None))
    for mesh, mx in meshes:
        for face in getattr(mesh,"faces",[]) or []:
            lp=list(getattr(face,"loop",[]) or [])
            if len(lp)<3: continue
            v0=_map_point(lp[0].position, mx)
            if not v0: continue
            for i in range(1,len(lp)-1):
                v1=_map_point(lp[i].position, mx); v2=_map_point(lp[i+1].position, mx)
                if not v1 or not v2: continue
                tris.append(Tri3D.from_points(v0,v1,v2,u_idx,v_idx,w_idx))
    return tris

def _infer_direction(proj_faces, w_idx) -> str:
    total = 0.0; count = 0
    for face, mx, _ in proj_faces:
        n = getattr(face, "normal", None)
        if n is None: continue
        try:
            nv = n if isinstance(n, Vec3) else qv_to_vec3(n)
            total += nv[w_idx]; count += 1
        except Exception:
            continue
    if count == 0: return "+"
    return "+" if total >= 0.0 else "-"

def _triangulate_polygon(poly) -> List[Polygon]:
    """Triangulate a polygon while respecting its boundary and holes."""
    if poly is None or poly.is_empty:
        return []
    if poly.geom_type == "MultiPolygon":
        tris = []
        for p in poly.geoms:
            tris.extend(_triangulate_polygon(p))
        return tris
    if poly.geom_type != "Polygon":
        return []

    tris: List[Polygon] = []

    if hasattr(poly, "constrained_delaunay_triangles"):
        try:
            cdt = poly.constrained_delaunay_triangles()
            geoms = cdt.geoms if hasattr(cdt, "geoms") else [cdt]
            for t in geoms:
                if t.geom_type == "Polygon" and t.area > 1e-12:
                    tris.append(t)
            if tris:
                return tris
        except Exception as e:
            dlog(f"constrained_delaunay_triangles failed: {e}; using fallback")

    try:
        tri_result = triangulate(poly)
        raw = tri_result if isinstance(tri_result, list) else list(tri_result.geoms)
    except Exception as e:
        dlog(f"triangulate failed: {e}")
        return []

    poly_buf = poly.buffer(1e-9)
    for t in raw:
        if t.geom_type != "Polygon" or t.area <= 1e-12:
            continue
        try:
            if poly_buf.contains(t):
                tris.append(t)
        except Exception:
            pass
    return tris


# -----------------------------------------------------------------------------
# Core: Surface Contour Project and Extrude
# -----------------------------------------------------------------------------
def _surface_contour_project_and_extrude(viewport, settings, unit_system: UnitSystem) -> bool:
    from PySide6.QtWidgets import QApplication
    scene = viewport.scene
    proj_faces, target_groups = _iter_selection(scene)

    if not proj_faces:
        ulog(viewport, "Invalid Selection", "No Cutter Face found. Use SUB-OBJECT (Face) mode.", QMessageBox.Icon.Warning); return False
    if len(target_groups) != 1:
        ulog(viewport, "Invalid Selection", f"Select exactly 1 Target Group. Found: {len(target_groups)}", QMessageBox.Icon.Warning); return False

    target = target_groups[0]
    lift = float(settings.get("lift", DEFAULT_LIFT))
    extrusion = float(settings.get("extrusion", DEFAULT_EXTRUSION))
    scale_factor = float(settings.get("scale", DEFAULT_SCALE))
    scale_after_extrude = bool(settings.get("scale_after_extrude", False))
    extra_scale = float(settings.get("extra_scale", DEFAULT_EXTRA_SCALE))
    cap_axis_enabled = bool(settings.get("cap_axis_enabled", False))
    cap_axis_scale = float(settings.get("cap_axis_scale", DEFAULT_CAP_AXIS_SCALE))
    axis = settings.get("axis", "Z")
    extrude_along = settings.get("extrude_along", "Projection Axis")

    w_idx = AXIS_IDX[axis]
    other = [i for i in range(3) if i != w_idx]
    u_idx, v_idx = other[0], other[1]

    axis_extrusion_vec = Vec3(0,0,0)
    if abs(extrusion) > 1e-9 and extrude_along == "Projection Axis":
        ev_arr = [0.0, 0.0, 0.0]; ev_arr[w_idx] = extrusion
        axis_extrusion_vec = Vec3(*ev_arr)

    direction = _infer_direction(proj_faces, w_idx)
    dlog(f"Inferred ray direction: '{direction}'")

    dlog(f"\n=== SURFACE CONTOUR PROJECT [{axis}] ===")
    dlog(f"Lift: {lift} | Extrusion: {extrusion} | Pre-scale: {scale_factor} | "
         f"Base scale: {extra_scale} (on={scale_after_extrude}) | "
         f"Cap axis scale: {cap_axis_scale} (on={cap_axis_enabled}) | Extrude Along: {extrude_along}")
    viewport.flash_status("Projection: Building Cutter...", 1000); QApplication.processEvents()

    # -----------------------------------------------------------------
    # Prepare per-face cutter data (polygon + its own plane height w)
    # -----------------------------------------------------------------
    face_data = []  # (poly, w_face, fidx)
    for fidx, (face, mx, _) in enumerate(proj_faces):
        poly = _get_cutter_polygon_2d(face, mx, u_idx, v_idx)
        if not poly or poly.is_empty or poly.area < 1e-12:
            continue
        ws = []
        for vtx in face.loop:
            wp = _map_point(vtx.position, mx)
            if wp: ws.append(wp[w_idx])
        w_face = sum(ws)/len(ws) if ws else 0.0
        face_data.append((poly, w_face, fidx))

    if not face_data:
        ulog(viewport, "Empty Cutter", "No valid cutter polygons found.", QMessageBox.Icon.Warning); return False

    # First scale: BEFORE projection (per-polygon, around each own centroid)
    scaled_face_data = []
    for poly, w_face, fidx in face_data:
        p = poly
        if abs(scale_factor - DEFAULT_SCALE) > 1e-9:
            centroid = p.centroid
            p = shapely_scale(p, xfact=scale_factor, yfact=scale_factor, origin=centroid)
            dlog(f"Face {fidx}: applied pre-projection scale {scale_factor} around {centroid}")
        if not p.is_empty and p.area > 1e-12:
            scaled_face_data.append((p, w_face, fidx))

    if not scaled_face_data:
        ulog(viewport, "Empty Cutter", "Cutter polygons became empty after scaling.", QMessageBox.Icon.Warning); return False

    # Target triangles (built once)
    viewport.flash_status("Projection: Collecting Target...", 2000); QApplication.processEvents()
    target_tris = _collect_target_triangles(target, u_idx, v_idx, w_idx)
    if not target_tris:
        ulog(viewport, "Empty Target", "Target Group has no geometry.", QMessageBox.Icon.Warning); return False

    # -----------------------------------------------------------------
    # Raycast per face (each uses its own w_face as the ray-origin plane)
    # Triangulation respects holes -> no faces generated inside holes.
    # -----------------------------------------------------------------
    viewport.flash_status("Projection: Shooting Rays (First Hit)...", 3000); QApplication.processEvents()

    def uv_to_world(u, v, w_plane):
        coords = [0.0, 0.0, 0.0]; coords[u_idx] = u; coords[v_idx] = v; coords[w_idx] = w_plane
        return Vec3(*coords)

    def try_direction(ray_dir):
        local_verts = []; local_normals = []; local_faces = []; local_cache = {}
        local_hits = 0; local_misses = 0

        def get_base_vert(u, v, w_plane, fidx):
            nonlocal local_hits, local_misses
            key = (round(u, 6), round(v, 6), fidx)
            if key in local_cache: return local_cache[key]
            pt_plane = uv_to_world(u, v, w_plane)
            ray_orig = pt_plane - ray_dir * RAY_FAR
            best_t = float('inf'); hit_pt = None; hit_normal = ray_dir
            for tri in target_tris:
                t = tri.intersect_ray(ray_orig, ray_dir)
                if t is not None and t < best_t:
                    best_t = t; hit_pt = ray_orig + ray_dir * t; hit_normal = tri.normal.normalized()
            final = (hit_pt + ray_dir * lift) if hit_pt else (pt_plane + ray_dir * lift)
            if hit_pt: local_hits += 1
            else: local_misses += 1
            idx = len(local_verts)
            local_verts.append(final); local_normals.append(hit_normal)
            local_cache[key] = idx
            return idx

        for poly, w_face, fidx in scaled_face_data:
            for tri in _triangulate_polygon(poly):
                coords = list(tri.exterior.coords)
                if len(coords) < 4: continue
                idxs = [get_base_vert(u, v, w_face, fidx) for u, v in coords[:3]]
                if len(set(idxs)) == 3:
                    local_faces.append(tuple(idxs))

        return local_verts, local_normals, local_faces, local_cache, local_hits, local_misses

    opp = "-" if direction == "+" else "+"
    base_verts = []; base_normals = []; base_faces = []; vert_cache = {}
    used_dir = direction; final_hits = 0; final_misses = 0
    for d in (direction, opp):
        ray_sign = 1.0 if d == "+" else -1.0
        fr = [0.0, 0.0, 0.0]; fr[w_idx] = ray_sign
        ray_dir = Vec3(*fr)
        b_verts, b_normals, b_faces, b_cache, b_hits, b_misses = try_direction(ray_dir)
        base_verts, base_normals, base_faces, vert_cache = b_verts, b_normals, b_faces, b_cache
        used_dir = d; final_hits = b_hits; final_misses = b_misses
        if b_hits > 0:
            break
        dlog(f"No hits with direction '{d}'; trying the opposite...")

    dlog(f"Raycast Done (dir '{used_dir}'). Hits: {final_hits}, Misses: {final_misses}, Base Faces: {len(base_faces)}")
    if not base_faces or final_hits == 0:
        ulog(viewport, "No Intersection", "No rays hit the target.", QMessageBox.Icon.Information); return False

    final_ray_sign = 1.0 if used_dir == "+" else -1.0
    fr = [0.0, 0.0, 0.0]; fr[w_idx] = final_ray_sign
    final_ray_dir = Vec3(*fr)

    # -----------------------------------------------------------------
    # Pre-cache exterior boundary vertices BEFORE fixing N (cap indexing depends on N)
    # -----------------------------------------------------------------
    def ensure_vertex(u, v, w_plane, fidx):
        key = (round(u, 6), round(v, 6), fidx)
        if key in vert_cache:
            return vert_cache[key]
        pt_plane = uv_to_world(u, v, w_plane)
        ray_orig = pt_plane - final_ray_dir * RAY_FAR
        best_t = float('inf'); hit_pt = None; hit_normal = final_ray_dir
        for tri in target_tris:
            t = tri.intersect_ray(ray_orig, final_ray_dir)
            if t is not None and t < best_t:
                best_t = t; hit_pt = ray_orig + final_ray_dir * t; hit_normal = tri.normal.normalized()
        final = (hit_pt + final_ray_dir * lift) if hit_pt else (pt_plane + final_ray_dir * lift)
        idx = len(base_verts)
        base_verts.append(final); base_normals.append(hit_normal)
        vert_cache[key] = idx
        return idx

    for poly, w_face, fidx in scaled_face_data:
        for u, v in list(poly.exterior.coords)[:-1]:
            ensure_vertex(u, v, w_face, fidx)

    # -----------------------------------------------------------------
    # Warning: cutter face(s) may be larger than the target surface
    # -----------------------------------------------------------------
    total_samples = final_hits + final_misses
    miss_ratio = (final_misses / total_samples) if total_samples > 0 else 0.0
    if miss_ratio > MISS_WARN_RATIO:
        msg = (f"About {miss_ratio*100:.1f}% of the cutter sample points did not hit the target surface.\n\n"
               "The cutter face(s) may be larger than the surface they project onto, which can cause "
               "incorrect geometry (floating or disconnected faces).\n\nContinue anyway?")
        reply = QMessageBox.warning(
            viewport.window(),
            "Cutter larger than target",
            msg,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            dlog("Aborted: cutter larger than target surface.")
            return False

    # -----------------------------------------------------------------
    # Build Mesh
    # -----------------------------------------------------------------
    viewport.flash_status("Projection: Building Mesh...", 1000)
    new_mesh = Mesh()

    do_extrude = abs(extrusion) > 1e-9
    N = len(base_verts)
    cap_verts = []

    if do_extrude:
        for i in range(N):
            if extrude_along == "Face Normal":
                offset = base_normals[i] * extrusion
            else:
                offset = axis_extrusion_vec
            cap_verts.append(base_verts[i] + offset)

    # Second scale: BASE ONLY (post-extrusion), using reference-face centroid as scale plane
    if scale_after_extrude and abs(extra_scale - DEFAULT_EXTRA_SCALE) > 1e-9 and base_verts:
        total_area = 0.0; cu = 0.0; cv = 0.0
        for poly, w_face, fidx in scaled_face_data:
            a = poly.area
            c = poly.centroid
            cu += c.x * a; cv += c.y * a; total_area += a
        if total_area > 0:
            cu /= total_area; cv /= total_area
            dlog(f"Applying post-extrusion base scale {extra_scale} around reference centroid ({cu:.6f}, {cv:.6f})")
            for i in range(N):
                u = base_verts[i][u_idx]
                v = base_verts[i][v_idx]
                w = base_verts[i][w_idx]
                coords = [0.0, 0.0, 0.0]
                coords[u_idx] = cu + (u - cu) * extra_scale
                coords[v_idx] = cv + (v - cv) * extra_scale
                coords[w_idx] = w
                base_verts[i] = Vec3(*coords)

    # Third scale: CAP AXIS SCALE — only the last face, only along projection axis
    if do_extrude and cap_axis_enabled and abs(cap_axis_scale - DEFAULT_CAP_AXIS_SCALE) > 1e-9:
        w_mean = sum(v[w_idx] for v in cap_verts) / len(cap_verts)
        dlog(f"Cap axis scale {cap_axis_scale} on axis {AXES[w_idx]} around w={w_mean:.6f} (cap only)")
        for i in range(len(cap_verts)):
            p = cap_verts[i]
            coords = [p.x, p.y, p.z]
            coords[w_idx] = w_mean + (coords[w_idx] - w_mean) * cap_axis_scale
            cap_verts[i] = Vec3(*coords)

    def add_face_safe(v_indices, flip=False):
        try:
            qv = []
            for i in v_indices:
                if i < N: qv.append(base_verts[i].to_qv())
                else: qv.append(cap_verts[i - N].to_qv())
            if flip: qv.reverse()
            new_mesh.add_face(qv, None)
        except Exception as e:
            dlog(f"Add Face Error: {e}")

    if not do_extrude:
        for i0, i1, i2 in base_faces:
            add_face_safe([i0, i1, i2])
    else:
        # Base
        for i0, i1, i2 in base_faces:
            add_face_safe([i0, i1, i2], flip=True)
        # Cap
        for i0, i1, i2 in base_faces:
            add_face_safe([i0+N, i1+N, i2+N], flip=False)

        # Walls ONLY from exterior rings of all scaled cutter faces.
        # No inner walls are generated for holes.
        def get_boundary_indices(ring_coords, fidx, w_face):
            indices = []
            for u, v in ring_coords[:-1]:
                indices.append(vert_cache[(round(u, 6), round(v, 6), fidx)])
            return indices

        def add_wall_quad(ia, ib, flip=False):
            ia_cap = ia + N; ib_cap = ib + N
            if flip: add_face_safe([ib, ia, ia_cap, ib_cap])
            else:    add_face_safe([ia, ib, ib_cap, ia_cap])

        for poly, w_face, fidx in scaled_face_data:
            ext_indices = get_boundary_indices(list(poly.exterior.coords), fidx, w_face)
            if len(ext_indices) < 2:
                continue
            for i in range(len(ext_indices)):
                add_wall_quad(ext_indices[i], ext_indices[(i+1) % len(ext_indices)], flip=False)

    # Hide Edges
    for e in list(getattr(new_mesh, "edges", [])) or []:
        try: e.hidden = True
        except: pass

    # Commit
    result_group_name = f"Projection_{target.name}"
    def mutate(scene):
        g = Group(new_mesh, name=result_group_name)
        try: g.layer = getattr(scene, "active_layer", None)
        except: pass
        scene.groups.append(g)
        try: scene.selection.clear(); scene.selection.add(g)
        except: pass
        scene.version = getattr(scene, "version", 0) + 1

    viewport.history.execute(SnapshotImport(mutate))
    if viewport.history.last_error:
        ulog(viewport, "History Error", str(viewport.history.last_error), QMessageBox.Icon.Critical); return False

    notify = getattr(viewport, "notify_scene_changed", None)
    if callable(notify): notify()
    viewport.update()

    msg = f"Success: {len(base_faces)} base faces"
    if do_extrude: msg += f" + Extrusion {extrusion:+.3f} ({extrude_along})"
    if abs(scale_factor - DEFAULT_SCALE) > 1e-9: msg += f" | Pre-scale {scale_factor:.2f}"
    if scale_after_extrude and abs(extra_scale - DEFAULT_EXTRA_SCALE) > 1e-9: msg += f" | Base scale {extra_scale:.2f}"
    if do_extrude and cap_axis_enabled and abs(cap_axis_scale - DEFAULT_CAP_AXIS_SCALE) > 1e-9: msg += f" | Cap axis scale {cap_axis_scale:.2f}"
    if miss_ratio > MISS_WARN_RATIO: msg += f" | Warning: {miss_ratio*100:.0f}% rays missed target"
    viewport.flash_status(msg, 5000)
    dlog("=== SURFACE CONTOUR PROJECT SUCCESS ===")
    return True


# -----------------------------------------------------------------------------
# UI Panel
# -----------------------------------------------------------------------------
class SurfaceContourProjectorPanel:
    def __init__(self, app):
        self.widget=QWidget(); self._app=app; self.unit_system = UnitSystem(app); self._build()

    def _build(self):
        root=QVBoxLayout(self.widget); root.setContentsMargins(8,8,8,8); root.setSpacing(8)

        unit_row = QHBoxLayout()
        unit_row.addWidget(QLabel("Units:"))
        self.unit_combo = QComboBox()
        self.unit_combo.addItems(list(self.unit_system.UNITS.keys()))
        self.unit_combo.setCurrentText(self.unit_system.current_key)
        self.unit_combo.currentTextChanged.connect(self._on_unit_changed)
        unit_row.addWidget(self.unit_combo)
        unit_row.addStretch(1)
        root.addLayout(unit_row)

        grp_proj = QGroupBox("Surface Contour Project and Extrude")
        lay_proj = QFormLayout(grp_proj)

        hint=QLabel(
            "<b>Selection:</b> Cutter Face(s) (Sub-Object) + Target Group (Object).<br>"
            "Multiple cutter faces are supported at once, including faces with holes.<br>"
            "• <b>Axis X/Y/Z</b>: Global projection axis.<br>"
            "• <b>Extrude Along</b>: Projection Axis or Face Normal (from target intersection).<br>"
            "• <b>Extrusion Distance</b>: Positive (Relief) or Negative (Recess). Zero = Single Face.<br>"
            "• <b>Scale Factor (Pre)</b>: Scales each cutter polygon around its own centroid BEFORE projection.<br>"
            "• <b>Scale Base After Extrusion</b>: Scales ONLY the base face afterwards, using the reference face "
            "centroid as the scale plane; the cap keeps the pre-scale (truncated pyramid).<br>"
            "• <b>Scale Cap Along Axis</b>: Scales ONLY the final extrusion face along the projection axis. "
            "1.0 = unchanged, 0 = planifies (flattens) the cap on that axis, &gt;1 = exaggerates the slope.<br>"
            "<i>Holes are preserved: no faces are generated inside the hollow areas and no inner walls are built. "
            "A warning is shown if the cutter appears larger than the target surface.</i>"
        )
        hint.setWordWrap(True); hint.setTextFormat(Qt.RichText); lay_proj.addRow(hint)

        self.axis=QComboBox(); self.axis.addItems(AXES); self.axis.setCurrentText("Z")
        lay_proj.addRow("Axis", self.axis)

        self.extrude_along=QComboBox(); self.extrude_along.addItems(EXTRUDE_ALONG); self.extrude_along.setCurrentText("Projection Axis")
        self.extrude_along.setToolTip(
            "Projection Axis: extrude along global X/Y/Z.<br>"
            "Face Normal: extrude along the normal of the TARGET face hit by the ray (projection intersection)."
        )
        lay_proj.addRow("Extrude Along", self.extrude_along)

        self.lift=QDoubleSpinBox(); self.lift.setRange(-10000,10000); self.lift.setValue(DEFAULT_LIFT)
        lay_proj.addRow("Lift (Offset)", self.lift)

        self.extrusion=QDoubleSpinBox(); self.extrusion.setRange(-100000,100000); self.extrusion.setValue(DEFAULT_EXTRUSION)
        self.extrusion.setToolTip("Type Positive (Relief) or Negative (Recess). Zero = Single Face.")
        lay_proj.addRow("Extrusion Distance", self.extrusion)

        self.scale=QDoubleSpinBox(); self.scale.setRange(0.01, 100.0); self.scale.setValue(DEFAULT_SCALE)
        self.scale.setDecimals(2); self.scale.setSingleStep(0.05)
        self.scale.setToolTip("1.0 = no change. < 1.0 shrinks the cutter; > 1.0 expands it. Applied before the raycast.")
        lay_proj.addRow("Scale Factor (Pre)", self.scale)

        self.scale_after_cb = QCheckBox("Scale Base After Extrusion")
        self.scale_after_cb.setChecked(False)
        self.scale_after_cb.setToolTip(
            "When enabled, applies a final scale ONLY to the base (reference) face after extrusion. "
            "The scale plane is the reference face centroid, keeping it coincident with the cutter face. "
            "The cap keeps the pre-projection scale, forming a truncated pyramid."
        )
        lay_proj.addRow(self.scale_after_cb)

        self.extra_scale = QDoubleSpinBox(); self.extra_scale.setRange(0.01, 100.0); self.extra_scale.setValue(DEFAULT_EXTRA_SCALE)
        self.extra_scale.setDecimals(2); self.extra_scale.setSingleStep(0.05)
        self.extra_scale.setEnabled(False)
        self.extra_scale.setToolTip("Factor applied only to the base after extrusion. 1.0 = no change.")
        self.scale_after_cb.toggled.connect(self.extra_scale.setEnabled)
        lay_proj.addRow("Post-extrusion Scale (Base)", self.extra_scale)

        self.cap_axis_cb = QCheckBox("Scale Cap Along Axis")
        self.cap_axis_cb.setChecked(False)
        self.cap_axis_cb.setToolTip(
            "When enabled, scales ONLY the last extrusion face (cap) along the projection axis. "
            "0 = planifies (flattens) the cap on that axis; >1 exaggerates the slope."
        )
        lay_proj.addRow(self.cap_axis_cb)

        self.cap_axis_scale = QDoubleSpinBox(); self.cap_axis_scale.setRange(0.0, 100.0); self.cap_axis_scale.setValue(DEFAULT_CAP_AXIS_SCALE)
        self.cap_axis_scale.setDecimals(2); self.cap_axis_scale.setSingleStep(0.05)
        self.cap_axis_scale.setEnabled(False)
        self.cap_axis_scale.setToolTip(
            "Factor applied only to the cap along the projection axis (X/Y/Z). "
            "1.0 = unchanged, 0 = planifies the cap, >1 = amplifies its slope."
        )
        self.cap_axis_cb.toggled.connect(self.cap_axis_scale.setEnabled)
        lay_proj.addRow("Cap Axis Scale", self.cap_axis_scale)

        self.btn_project=QPushButton("Project and Extrude")
        self.btn_project.setStyleSheet("font-weight: bold; padding: 10px; background-color: #3a7; color: white; font-size: 13px;")
        self.btn_project.clicked.connect(self._do_project)
        lay_proj.addRow(self.btn_project)
        root.addWidget(grp_proj)

        self.status=QLabel("Ready. Select Face(s) + 1 Target Group.")
        self.status.setWordWrap(True); root.addWidget(self.status)
        root.addStretch(1)

        self._update_ui_from_units()

    def _on_unit_changed(self, key: str):
        self.unit_system.set_unit(key)
        self._update_ui_from_units()

    def _update_ui_from_units(self):
        props = self.unit_system.get_props()
        suffix = props["suffix"]; decimals = props["decimals"]; step = props["step"]
        for spin in [self.lift, self.extrusion]:
            spin.setSuffix(suffix); spin.setDecimals(decimals); spin.setSingleStep(step)
            factor = props["factor"]; max_val = 10000 / factor if factor > 0 else 10000
            spin.setRange(-max_val, max_val)

    def refresh(self):
        try:
            pf,tg=_iter_selection(self._app.scene)
            self.status.setText(f"Selection: {len(pf)} Cutter Face(s), {len(tg)} Target Group(s).")
            self.btn_project.setEnabled(len(pf)>0 and len(tg)==1)
        except: self.btn_project.setEnabled(False)

    def _do_project(self):
        settings={
            "axis": self.axis.currentText(),
            "extrude_along": self.extrude_along.currentText(),
            "lift": self.lift.value(),
            "extrusion": self.extrusion.value(),
            "scale": self.scale.value(),
            "scale_after_extrude": self.scale_after_cb.isChecked(),
            "extra_scale": self.extra_scale.value(),
            "cap_axis_enabled": self.cap_axis_cb.isChecked(),
            "cap_axis_scale": self.cap_axis_scale.value()
        }
        _surface_contour_project_and_extrude(self._app.viewport, settings, self.unit_system)
        self.refresh()


# -----------------------------------------------------------------------------
# Plugin Entry
# -----------------------------------------------------------------------------
def setup(app):
    dlog(">>> Plugin setup() Surface Contour Projector and Extruder")
    if not SHAPELY_OK: QMessageBox.critical(None, PLUGIN_NAME, f"Shapely: {SHAPELY_ERR}"); return
    if not CORE_OK: QMessageBox.critical(None, PLUGIN_NAME, f"Core: {CORE_ERR}"); return
    try:
        panel=SurfaceContourProjectorPanel(app)
        dock=app.add_panel(PLUGIN_NAME, panel.widget, name="surface_contour_projector")
        app.add_menu_action("Surface Contour Projector and Extruder…", lambda: app.show_panel(dock), tip="Project cutter contours onto a target surface and extrude them into slopes, curbs, sidewalks, and markings.")
        app.on_document_changed(panel.refresh)
        panel.refresh()
    except Exception as e:
        dlog(f"SETUP CRASH: {traceback.format_exc()}")
        QMessageBox.critical(None, PLUGIN_NAME, f"Load Error: {e}")