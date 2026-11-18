"""Print joint limits, drive gains and max joint velocities authored in each given USD layer, layer by layer (read-only;
G1 gaps, #106: which robot file carries which joint ranges). Usage: CUDA_VISIBLE_DEVICES="" ... /isaac-sim/python.sh sdf_limits.py <layer> [...]
"""
import sys
from pxr import Sdf
for p in sys.argv[1:]:
    lay = Sdf.Layer.FindOrOpen(p)
    print("====", p, "sublayers", list(lay.subLayerPaths))
    def visit(path):
        spec = lay.GetObjectAtPath(path)
        if isinstance(spec, Sdf.AttributeSpec):
            n = spec.name
            if any(k in n for k in ("Limit", "limit", "maxJointVelocity", "stiffness", "damping", "maxForce")):
                print(" ", path, spec.default)
    lay.Traverse(Sdf.Path.absoluteRootPath, visit)
