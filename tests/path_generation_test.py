"""
Robust STEP edge sampling and Matplotlib wireframe plot.

Handles variations of pythonocc-core / OCCT API where
BRep_Tool.Curve may return different types (tuple, handle, etc.)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.BRep import BRep_Tool
from OCC.Core.gp import gp_Pnt
import warnings

def _extract_curve_handle(maybe_curve):
    """
    Normalize the return from BRep_Tool.Curve(edge) into an object
    that supports D0(param, gp_Pnt) or None.
    """
    if maybe_curve is None:
        return None

    # If it's already an object with D0, return it
    if hasattr(maybe_curve, "D0"):
        return maybe_curve

    # If it's a tuple/list, attempt to find an element with D0
    if isinstance(maybe_curve, (tuple, list)):
        for item in maybe_curve:
            if item is None:
                continue
            if hasattr(item, "D0"):
                return item
            # Some handles wrap the object; try GetObject / Get / get
            if hasattr(item, "GetObject"):
                obj = item.GetObject()
                if hasattr(obj, "D0"):
                    return obj
            if hasattr(item, "Get"):
                obj = item.Get()
                if hasattr(obj, "D0"):
                    return obj
    # If it has GetObject/Get but not D0 directly
    if hasattr(maybe_curve, "GetObject"):
        obj = maybe_curve.GetObject()
        if hasattr(obj, "D0"):
            return obj
    if hasattr(maybe_curve, "Get"):
        obj = maybe_curve.Get()
        if hasattr(obj, "D0"):
            return obj

    # Last resort: return None (we couldn't find a usable curve)
    return None

def sample_edge_points(edge, n_points=10):
    """Return sampled XYZ points along an edge robust to pythonocc variations."""
    # Attempt to get curve (may be handle, tuple, or None)
    maybe_curve = None
    try:
        maybe_curve = BRep_Tool.Curve(edge)
    except Exception:
        # Some pythonocc builds raise; ignore and try other routes below
        maybe_curve = None

    curve = _extract_curve_handle(maybe_curve)

    # If we still didn't find a curve, bail out gracefully
    if curve is None:
        warnings.warn("Could not extract Geom_Curve from edge; skipping this edge.")
        return np.zeros((0, 3))

    # Get parameter range for the edge
    try:
        first, last = BRep_Tool.Range(edge)
    except Exception:
        # If Range fails, try to extract from maybe_curve if it had (curve, first, last)
        first = None
        last = None
        if isinstance(maybe_curve, (tuple, list)) and len(maybe_curve) >= 3:
            # often (curve, first, last)
            _, f, l = maybe_curve[:3]
            first, last = f, l

    if first is None or last is None:
        warnings.warn("Could not determine parameter range for edge; using default [0,1].")
        first, last = 0.0, 1.0

    # Now sample points along the curve using D0
    pts = []
    for i in range(n_points):
        t = first + (last - first) * (i / (n_points - 1))
        p = gp_Pnt()
        # curve may still be a handle object where we need to call .D0 on the underlying object
        try:
            curve.D0(t, p)
        except Exception:
            # try calling D0 on GetObject() / Get()
            if hasattr(curve, "GetObject"):
                obj = curve.GetObject()
                obj.D0(t, p)
            elif hasattr(curve, "Get"):
                obj = curve.Get()
                obj.D0(t, p)
            else:
                # give up for this edge
                warnings.warn("Failed to call D0 on curve object at param %s" % t)
                return np.zeros((0, 3))
        pts.append([p.X(), p.Y(), p.Z()])

    return np.array(pts)

def load_step_edges(filename):
    """Load all CAD edges from a STEP file."""
    reader = STEPControl_Reader()
    status = reader.ReadFile(filename)
    if status != 1:
        raise RuntimeError(f"Cannot read STEP file: {filename} (status={status})")
    reader.TransferRoots()
    shape = reader.OneShape()

    exp = TopExp_Explorer(shape, TopAbs_EDGE)
    edges = []
    while exp.More():
        edges.append(exp.Current())
        exp.Next()
    return edges

def plot_edges(edges, sampling=10):
    """Plot all edges using Matplotlib with true aspect ratio."""
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection='3d')

    segments = []
    all_points = []

    for edge in edges:
        pts = sample_edge_points(edge, n_points=sampling)
        if pts.shape[0] == 0:
            continue
        for i in range(len(pts) - 1):
            segments.append([pts[i], pts[i + 1]])
        all_points.append(pts)

    if not segments:
        raise RuntimeError("No valid edge segments to plot.")

    lc = Line3DCollection(segments, colors='k', linewidths=0.6)
    ax.add_collection3d(lc)

    # Combine all points for scaling
    all_pts = np.vstack(all_points)
    x_min, y_min, z_min = np.min(all_pts, axis=0)
    x_max, y_max, z_max = np.max(all_pts, axis=0)

    # Compute ranges
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)

    # Center model and set equal aspect ratio
    mid_x = (x_max + x_min) / 2
    mid_y = (y_max + y_min) / 2
    mid_z = (z_max + z_min) / 2

    ax.set_xlim(mid_x - max_range / 2, mid_x + max_range / 2)
    ax.set_ylim(mid_y - max_range / 2, mid_y + max_range / 2)
    ax.set_zlim(mid_z - max_range / 2, mid_z + max_range / 2)

    # Set cubic box aspect ratio (equal scaling)
    ax.set_box_aspect([1, 1, 1])
    ax.set_axis_off()
    plt.tight_layout()
    plt.show()
    return segments

if __name__ == "__main__":
    filename = "./data/UCLAMAE.step"  # Replace with your STEP file path
    edges = load_step_edges(filename)
    segments = plot_edges(edges, sampling=10)
    # segments = (sampling_num * 2 - 2) * read_edges
    with open("./data/segments/UCLAMAE_segments.txt", "w") as f:
        for seg in segments:
            f.write(f"{seg[0][0]}, {seg[0][1]}, {seg[0][2]}, {seg[1][0]}, {seg[1][1]}, {seg[1][2]}\n")
