import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import warnings

# libraries from pythonocc
from OCC.Core.STEPControl import STEPControl_Reader
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_EDGE
from OCC.Core.BRep import BRep_Tool
from OCC.Core.gp import gp_Pnt


class PathGenerator:
    def __init__(self, file_path, sampling=10):
        """
        Initialize the PathGenerator.
        :param sampling: number of sample points per edge
        """
        self.file_path = file_path
        self.sampling = sampling

    # -------------------- Private Helpers --------------------

    def _extract_curve_handle(self, maybe_curve):
        """
        Normalize BRep_Tool.Curve(edge) result into an object supporting D0(param, gp_Pnt).
        """
        if maybe_curve is None:
            return None

        # Directly usable
        if hasattr(maybe_curve, "D0"):
            return maybe_curve

        # If it's a tuple/list, try to find a valid curve object inside
        if isinstance(maybe_curve, (tuple, list)):
            for item in maybe_curve:
                if item is None:
                    continue
                if hasattr(item, "D0"):
                    return item
                if hasattr(item, "GetObject"):
                    obj = item.GetObject()
                    if hasattr(obj, "D0"):
                        return obj
                if hasattr(item, "Get"):
                    obj = item.Get()
                    if hasattr(obj, "D0"):
                        return obj

        # Try handle wrappers
        if hasattr(maybe_curve, "GetObject"):
            obj = maybe_curve.GetObject()
            if hasattr(obj, "D0"):
                return obj
        if hasattr(maybe_curve, "Get"):
            obj = maybe_curve.Get()
            if hasattr(obj, "D0"):
                return obj

        return None

    def _sample_edge_points(self, edge):
        try:
            maybe_curve = BRep_Tool.Curve(edge)
        except Exception:
            maybe_curve = None

        curve = self._extract_curve_handle(maybe_curve)

        if curve is None:
            warnings.warn("Could not extract Geom_Curve from edge; skipping this edge.")
            return np.zeros((0, 3))

        # Try to get parameter range
        try:
            first, last = BRep_Tool.Range(edge)
        except Exception:
            first = last = None
            if isinstance(maybe_curve, (tuple, list)) and len(maybe_curve) >= 3:
                _, f, l = maybe_curve[:3]
                first, last = f, l

        if first is None or last is None:
            warnings.warn("Could not determine parameter range for edge; using [0,1].")
            first, last = 0.0, 1.0

        pts = []
        for i in range(self.sampling):
            t = first + (last - first) * (i / (self.sampling - 1))
            p = gp_Pnt()
            try:
                curve.D0(t, p)
            except Exception:
                if hasattr(curve, "GetObject"):
                    curve.GetObject().D0(t, p)
                elif hasattr(curve, "Get"):
                    curve.Get().D0(t, p)
                else:
                    warnings.warn(f"Failed to call D0 on curve object at param {t}")
                    return np.zeros((0, 3))
            pts.append([p.X(), p.Y(), p.Z()])

        return np.array(pts)

    # -------------------- Public API --------------------

    def load_step_edges(self, filename):
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

    def edges_to_segments(self, edges):

        segments = []
        all_points = []

        for edge in edges:
            pts = self._sample_edge_points(edge)
            if pts.shape[0] == 0:
                continue
            for i in range(len(pts) - 1):
                segments.append([pts[i], pts[i + 1]])
            all_points.append(pts)

        return segments

    def generate_from_step(self, segment_file=None):
        edges = self.load_step_edges(self.file_path)
        segments = self.edges_to_segments(edges)

        if segment_file:
            with open(segment_file, "w") as f:
                for seg in segments:
                    f.write(
                        f"{seg[0][0]}, {seg[0][1]}, {seg[0][2]}, "
                        f"{seg[1][0]}, {seg[1][1]}, {seg[1][2]}\n"
                    )
        return segments