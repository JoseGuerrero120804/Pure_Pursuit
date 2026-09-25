#!/usr/bin/env python3
import argparse
import csv
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from pure_pursuit.path_utils import classify_segments


def load_xy(csv_file):
    with open(csv_file, newline='') as f:
        reader = csv.DictReader(f)
        points = [(float(row['x']), float(row['y'])) for row in reader]
    return np.array(points)


def cross_track_errors(actual, reference):
    """For each actual-trajectory point, the distance to the closest segment of
    the reference path (point-to-segment distance, not just nearest vertex) and
    the index of that nearest segment."""
    a = reference[:-1]
    b = reference[1:]
    ab = b - a
    ab_len2 = np.sum(ab ** 2, axis=1)
    ab_len2_safe = np.where(ab_len2 == 0, 1.0, ab_len2)

    errors = np.empty(len(actual))
    nearest_segment = np.empty(len(actual), dtype=int)
    for i, p in enumerate(actual):
        ap = p - a
        t = np.clip(np.sum(ap * ab, axis=1) / ab_len2_safe, 0.0, 1.0)
        proj = a + t[:, None] * ab
        dists = np.linalg.norm(p - proj, axis=1)
        j = int(np.argmin(dists))
        errors[i] = dists[j]
        nearest_segment[i] = j
    return errors, nearest_segment


def load_controller_tuning(params_file):
    """Read the look-ahead / speed tuning values from the controller's params
    YAML, so the report reflects what was actually used for the run (rather
    than a copy that could drift out of sync)."""
    if not params_file or not os.path.isfile(params_file):
        return None
    with open(params_file) as f:
        data = yaml.safe_load(f)
    try:
        return data['pure_pursuit_controller']['ros__parameters']
    except (KeyError, TypeError):
        return None


def main():
    parser = argparse.ArgumentParser(
        description='Compare a recorded reference path against an executed trajectory: '
                     'compute cross-track error (overall and curve-vs-straight) and plot '
                     'both overlaid.'
    )
    parser.add_argument('--reference', default=os.path.expanduser('~/waypoints.csv'),
                         help='Reference path CSV (from path_recorder during teleop).')
    parser.add_argument('--actual', default=os.path.expanduser('~/actual_trajectory.csv'),
                         help='Executed trajectory CSV (from path_recorder during autonomous tracking).')
    parser.add_argument('--output', default=os.path.expanduser('~/trajectory_comparison.png'),
                         help='Output path for the overlay plot.')
    parser.add_argument('--controller-params', default=None,
                         help='Path to the pure_pursuit_controller params YAML used for the '
                              'run (defaults to the package-installed pure_pursuit_params.yaml), '
                              'used only to report the look-ahead/speed tuning values.')
    parser.add_argument('--curve-curvature-threshold', type=float, default=None,
                         help='Curvature threshold (rad/m) used to classify a path segment '
                              'as a curve. Defaults to the controller params value, or 0.1.')
    args = parser.parse_args()

    controller_params_file = args.controller_params
    if controller_params_file is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            controller_params_file = os.path.join(
                get_package_share_directory('pure_pursuit'), 'config', 'pure_pursuit_params.yaml'
            )
        except Exception:
            controller_params_file = None
    tuning = load_controller_tuning(controller_params_file) or {}

    curve_curvature_threshold = args.curve_curvature_threshold
    if curve_curvature_threshold is None:
        curve_curvature_threshold = tuning.get('curve_curvature_threshold', 0.1)

    reference = load_xy(args.reference)
    actual = load_xy(args.actual)

    errors, nearest_segment = cross_track_errors(actual, reference)
    seg_is_curve = np.array(classify_segments(reference, curve_curvature_threshold))
    point_is_curve = seg_is_curve[nearest_segment]

    mean_error = errors.mean()
    max_error = errors.max()
    curve_errors = errors[point_is_curve]
    straight_errors = errors[~point_is_curve]

    print(f'Reference waypoints : {len(reference)}')
    print(f'Actual waypoints    : {len(actual)}')
    print(f'Mean cross-track error (overall)  : {mean_error:.4f} m')
    print(f'Max cross-track error (overall)   : {max_error:.4f} m')
    if len(straight_errors):
        print(f'Mean cross-track error (straights): {straight_errors.mean():.4f} m '
              f'over {len(straight_errors)} pts')
    else:
        print('Mean cross-track error (straights): n/a (no straight points matched)')
    if len(curve_errors):
        print(f'Mean cross-track error (curves)   : {curve_errors.mean():.4f} m '
              f'over {len(curve_errors)} pts')
    else:
        print('Mean cross-track error (curves)   : n/a (no curve points matched)')

    k = tuning.get('lookahead_gain')
    p_min = tuning.get('min_lookahead_distance')
    v_straight = tuning.get('linear_velocity')
    v_curve = tuning.get('linear_velocity_curve')
    tuning_line = None
    if k is not None:
        print(f'\nLook-ahead tuning: k={k}, curve_curvature_threshold={curve_curvature_threshold} rad')
        tuning_parts = [f'k={k}']
        if v_straight is not None and p_min is not None:
            P_straight = max(k * v_straight, p_min)
            print(f'  v_straight={v_straight} m/s -> P_straight={P_straight:.3f} m')
            tuning_parts.append(f'P_straight={P_straight:.2f} m @ v={v_straight} m/s')
        if v_curve is not None and p_min is not None:
            P_curve = max(k * v_curve, p_min)
            print(f'  v_curve={v_curve} m/s -> P_curve={P_curve:.3f} m')
            tuning_parts.append(f'P_curve={P_curve:.2f} m @ v={v_curve} m/s')
        tuning_line = ', '.join(tuning_parts)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.plot(reference[:, 0], reference[:, 1], 'b-', linewidth=2, label='Reference path')
    if seg_is_curve.any():
        curve_pts = reference[:-1][seg_is_curve]
        ax.scatter(curve_pts[:, 0], curve_pts[:, 1], c='orange', s=10, zorder=3,
                   label='Reference curve segments')
    ax.plot(actual[:, 0], actual[:, 1], 'r--', linewidth=2, label='Executed trajectory')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    title = (f'Pure Pursuit Tracking\n'
             f'Mean cross-track error: {mean_error:.3f} m '
             f'(straight: {straight_errors.mean() if len(straight_errors) else float("nan"):.3f} m, '
             f'curve: {curve_errors.mean() if len(curve_errors) else float("nan"):.3f} m)')
    if tuning_line:
        title += f'\n{tuning_line}'
    ax.set_title(title)
    ax.legend()
    ax.axis('equal')
    ax.grid(True)

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.savefig(args.output, dpi=300)
    print(f'\nOverlay plot saved to {args.output}')


if __name__ == '__main__':
    main()
