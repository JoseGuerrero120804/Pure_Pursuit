import math


def path_curvatures(path):
    """Local curvature (rad/m) at each point: heading change between consecutive
    segments divided by segment length. Using curvature rather than raw heading
    change makes the value independent of how finely the path was sampled
    (a tight turn recorded with closely-spaced points has small per-step heading
    changes but the same curvature as one recorded with sparser points).
    Endpoints (undefined) are 0.0."""
    n = len(path)
    curvatures = [0.0] * n
    for i in range(1, n - 1):
        x0, y0 = path[i - 1]
        x1, y1 = path[i]
        x2, y2 = path[i + 1]
        h1 = math.atan2(y1 - y0, x1 - x0)
        h2 = math.atan2(y2 - y1, x2 - x1)
        dh = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
        seg_len = math.hypot(x2 - x1, y2 - y1)
        curvatures[i] = dh / seg_len if seg_len > 1e-9 else 0.0
    return curvatures


def classify_segments(path, curvature_threshold):
    """Boolean list of length len(path)-1: True if segment i->i+1 belongs to a
    curve (either endpoint's local curvature exceeds curvature_threshold, rad/m)."""
    point_curvatures = path_curvatures(path)
    n = len(path)
    return [
        point_curvatures[i] >= curvature_threshold or point_curvatures[i + 1] >= curvature_threshold
        for i in range(n - 1)
    ]
