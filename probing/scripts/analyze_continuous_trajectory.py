#!/usr/bin/env python3

import argparse
import csv
import math
from collections import OrderedDict


POSITION_FIELDS = ("tip_x_m", "tip_y_m", "tip_z_m")
QUATERNION_FIELDS = ("tip_qx", "tip_qy", "tip_qz", "tip_qw")


def parse_float(row, field):
    value = row.get(field, "")
    return float(value) if value not in (None, "") else None


def load_stage_samples(path):
    stages = OrderedDict()
    seen_stamps = {}
    with open(path, newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            stage = row.get("stage", "")
            stamp_s = parse_float(row, "tip_stamp_s")
            position = tuple(parse_float(row, field) for field in POSITION_FIELDS)
            quaternion = tuple(
                parse_float(row, field) for field in QUATERNION_FIELDS
            )
            speed_limit_mps = parse_float(row, "cartesian_speed_limit_m_s")
            angular_speed_limit_radps = parse_float(
                row,
                "cartesian_angular_speed_limit_rad_s",
            )
            if not stage or stamp_s is None or any(value is None for value in position):
                continue
            if stamp_s in seen_stamps.setdefault(stage, set()):
                continue
            seen_stamps[stage].add(stamp_s)
            stages.setdefault(stage, []).append(
                {
                    "stamp_s": stamp_s,
                    "position": position,
                    "quaternion": quaternion,
                    "speed_limit_mps": speed_limit_mps,
                    "angular_speed_limit_radps": angular_speed_limit_radps,
                }
            )
    return stages


def distance(first, second):
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))


def point_line_distance(point, start, end):
    direction = tuple(b - a for a, b in zip(start, end))
    direction_norm_sq = sum(value * value for value in direction)
    if direction_norm_sq == 0.0:
        return distance(point, start)
    offset = tuple(value - origin for value, origin in zip(point, start))
    fraction = sum(a * b for a, b in zip(offset, direction)) / direction_norm_sq
    projection = tuple(
        origin + fraction * delta for origin, delta in zip(start, direction)
    )
    return distance(point, projection)


def quaternion_angle(first, second):
    if any(value is None for value in first + second):
        return None
    first_norm = math.sqrt(sum(value * value for value in first))
    second_norm = math.sqrt(sum(value * value for value in second))
    if first_norm == 0.0 or second_norm == 0.0:
        return None
    dot = abs(sum(a * b for a, b in zip(first, second))) / (
        first_norm * second_norm
    )
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return None
    index = fraction * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def analyze_stage(samples):
    positions = [sample["position"] for sample in samples]
    start = positions[0]
    end = positions[-1]
    segment_lengths = [
        distance(first, second) for first, second in zip(positions, positions[1:])
    ]
    line_errors = [point_line_distance(point, start, end) for point in positions]
    speeds = []
    for first, second, segment_length in zip(samples, samples[1:], segment_lengths):
        duration_s = second["stamp_s"] - first["stamp_s"]
        if duration_s > 0.0 and segment_length >= 1e-5:
            speeds.append(segment_length / duration_s)

    start_quaternion = samples[0]["quaternion"]
    orientation_changes = [
        angle
        for sample in samples
        if (angle := quaternion_angle(start_quaternion, sample["quaternion"]))
        is not None
    ]
    angular_segment_lengths = [
        quaternion_angle(first["quaternion"], second["quaternion"]) or 0.0
        for first, second in zip(samples, samples[1:])
    ]
    cumulative_distances = [0.0]
    for segment_length in segment_lengths:
        cumulative_distances.append(cumulative_distances[-1] + segment_length)
    stable_speeds = []
    total_path_length = cumulative_distances[-1]
    # Long stages use a wider window to suppress TF timestamp jitter. Short
    # stages need a narrower window so acceleration does not enter the cruise
    # estimate.
    speed_window = 3 if total_path_length < 0.01 else 5
    if len(samples) > speed_window and total_path_length > 0.0:
        for index in range(len(samples) - speed_window):
            end_index = index + speed_window
            duration_s = (
                samples[end_index]["stamp_s"] - samples[index]["stamp_s"]
            )
            distance_m = (
                cumulative_distances[end_index] - cumulative_distances[index]
            )
            midpoint_fraction = 0.5 * (
                cumulative_distances[index] + cumulative_distances[end_index]
            ) / total_path_length
            if duration_s > 0.0 and 0.3 <= midpoint_fraction <= 0.7:
                stable_speeds.append(distance_m / duration_s)

    stable_speed_mean = (
        sum(stable_speeds) / len(stable_speeds) if stable_speeds else None
    )
    stable_speed_cv = None
    if stable_speed_mean is not None and stable_speed_mean > 0.0:
        variance = sum(
            (value - stable_speed_mean) ** 2 for value in stable_speeds
        ) / len(stable_speeds)
        stable_speed_cv = math.sqrt(variance) / stable_speed_mean

    cumulative_angles = [0.0]
    for segment_angle in angular_segment_lengths:
        cumulative_angles.append(cumulative_angles[-1] + segment_angle)
    stable_angular_speeds = []
    total_angle = cumulative_angles[-1]
    if len(samples) > speed_window and total_angle > 0.0:
        for index in range(len(samples) - speed_window):
            end_index = index + speed_window
            duration_s = (
                samples[end_index]["stamp_s"] - samples[index]["stamp_s"]
            )
            angle_rad = cumulative_angles[end_index] - cumulative_angles[index]
            midpoint_fraction = 0.5 * (
                cumulative_angles[index] + cumulative_angles[end_index]
            ) / total_angle
            if duration_s > 0.0 and 0.3 <= midpoint_fraction <= 0.7:
                stable_angular_speeds.append(angle_rad / duration_s)
    stable_angular_mean = (
        sum(stable_angular_speeds) / len(stable_angular_speeds)
        if stable_angular_speeds
        else None
    )
    stable_angular_cv = None
    if stable_angular_mean is not None and stable_angular_mean > 0.0:
        variance = sum(
            (value - stable_angular_mean) ** 2
            for value in stable_angular_speeds
        ) / len(stable_angular_speeds)
        stable_angular_cv = math.sqrt(variance) / stable_angular_mean

    return {
        "samples": len(samples),
        "duration_s": samples[-1]["stamp_s"] - samples[0]["stamp_s"],
        "displacement_m": distance(start, end),
        "path_length_m": sum(segment_lengths),
        "line_max_m": max(line_errors, default=0.0),
        "line_rms_m": math.sqrt(
            sum(value * value for value in line_errors) / len(line_errors)
        ),
        "speed_median_mps": percentile(speeds, 0.5),
        "speed_min_mps": min(speeds, default=None),
        "speed_max_mps": max(speeds, default=None),
        "stable_speed_median_mps": percentile(stable_speeds, 0.5),
        "stable_speed_mean_mps": stable_speed_mean,
        "stable_speed_p10_mps": percentile(stable_speeds, 0.1),
        "stable_speed_p90_mps": percentile(stable_speeds, 0.9),
        "stable_speed_cv": stable_speed_cv,
        "speed_limit_mps": next(
            (
                sample["speed_limit_mps"]
                for sample in samples
                if sample["speed_limit_mps"] is not None
            ),
            None,
        ),
        "stable_angular_median_radps": percentile(stable_angular_speeds, 0.5),
        "stable_angular_mean_radps": stable_angular_mean,
        "stable_angular_p10_radps": percentile(stable_angular_speeds, 0.1),
        "stable_angular_p90_radps": percentile(stable_angular_speeds, 0.9),
        "stable_angular_cv": stable_angular_cv,
        "angular_speed_limit_radps": next(
            (
                sample["angular_speed_limit_radps"]
                for sample in samples
                if sample["angular_speed_limit_radps"] is not None
            ),
            None,
        ),
        "angular_path_rad": total_angle,
        "orientation_max_rad": max(orientation_changes, default=None),
        "speed_window": speed_window,
    }


def format_speed(value):
    return "n/a" if value is None else f"{value * 1000.0:.2f}"


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Analyze continuously sampled probing TCP telemetry. Repeated TF "
            "timestamps are removed before calculating geometry and speed."
        )
    )
    parser.add_argument("csv_path", help="Telemetry CSV produced by run_fr3_probe.py")
    parser.add_argument(
        "--max-cruise-error-pct",
        type=float,
        default=5.0,
        help="Maximum absolute cruise-speed error for acceptance.",
    )
    parser.add_argument(
        "--max-cruise-cv-pct",
        type=float,
        default=10.0,
        help="Maximum cruise-speed coefficient of variation for acceptance.",
    )
    parser.add_argument(
        "--fail-on-speed-error",
        action="store_true",
        help="Exit nonzero when any measured cruise stage exceeds a limit.",
    )
    args = parser.parse_args()

    stages = load_stage_samples(args.csv_path)
    if not stages:
        raise SystemExit("No timestamped TCP samples found in the telemetry CSV.")

    print(
        "stage              samples duration  displacement path      "
        "line max/rms    speed median[min,max]  orientation max"
    )
    speed_checks = []
    for stage, samples in stages.items():
        if len(samples) < 2:
            print(f"{stage:<18} {len(samples):>7} insufficient samples")
            continue
        result = analyze_stage(samples)
        orientation_deg = (
            "n/a"
            if result["orientation_max_rad"] is None
            else f'{math.degrees(result["orientation_max_rad"]):.2f} deg'
        )
        print(
            f"{stage:<18} {result['samples']:>7} "
            f"{result['duration_s']:>7.3f}s "
            f"{result['displacement_m'] * 1000.0:>8.2f}mm "
            f"{result['path_length_m'] * 1000.0:>8.2f}mm "
            f"{result['line_max_m'] * 1000.0:>6.2f}/"
            f"{result['line_rms_m'] * 1000.0:<6.2f}mm "
            f"{format_speed(result['speed_median_mps']):>7}"
            f"[{format_speed(result['speed_min_mps'])},"
            f"{format_speed(result['speed_max_mps'])}] mm/s "
            f"{orientation_deg}"
        )
        is_angular_stage = (
            result["angular_path_rad"] >= 0.01
            and result["path_length_m"] < 0.001
        )
        stable_median = result["stable_speed_median_mps"]
        if (
            stable_median is not None
            and result["path_length_m"] >= 1e-4
            and not is_angular_stage
            and stage not in ("APPROACH", "HOLD")
        ):
            target = result["speed_limit_mps"]
            target_text = "n/a"
            error_text = "n/a"
            if target is not None and target > 0.0:
                target_text = f"{target * 1000.0:.2f}"
                error_pct = 100.0 * (stable_median / target - 1.0)
                error_text = f"{error_pct:+.2f}%"
                speed_checks.append(
                    (stage, "linear", error_pct, result["stable_speed_cv"] * 100.0)
                )
            print(
                f"  cruise 30-70% ({result['speed_window']}-TF window): "
                f"median={stable_median * 1000.0:.2f}, "
                f"mean={result['stable_speed_mean_mps'] * 1000.0:.2f}, "
                f"p10-p90={result['stable_speed_p10_mps'] * 1000.0:.2f}-"
                f"{result['stable_speed_p90_mps'] * 1000.0:.2f} mm/s, "
                f"CV={result['stable_speed_cv'] * 100.0:.2f}%, "
                f"target={target_text} mm/s, error={error_text}"
            )
        stable_angular_median = result["stable_angular_median_radps"]
        if (
            stable_angular_median is not None
            and is_angular_stage
            and stage not in ("APPROACH", "HOLD")
        ):
            target = result["angular_speed_limit_radps"]
            target_text = "n/a"
            error_text = "n/a"
            if target is not None and target > 0.0:
                target_text = f"{target:.3f}"
                error_pct = 100.0 * (stable_angular_median / target - 1.0)
                error_text = f"{error_pct:+.2f}%"
                speed_checks.append(
                    (
                        stage,
                        "angular",
                        error_pct,
                        result["stable_angular_cv"] * 100.0,
                    )
                )
            print(
                f"  angular cruise 30-70% ({result['speed_window']}-TF window): "
                f"median={stable_angular_median:.3f}, "
                f"mean={result['stable_angular_mean_radps']:.3f}, "
                f"p10-p90={result['stable_angular_p10_radps']:.3f}-"
                f"{result['stable_angular_p90_radps']:.3f} rad/s, "
                f"CV={result['stable_angular_cv'] * 100.0:.2f}%, "
                f"target={target_text} rad/s, error={error_text}"
            )

    failures = []
    if speed_checks:
        print("speed acceptance:")
    for stage, mode, error_pct, cv_pct in speed_checks:
        passed = (
            abs(error_pct) <= args.max_cruise_error_pct
            and cv_pct <= args.max_cruise_cv_pct
        )
        print(
            f"  {'PASS' if passed else 'FAIL'} {stage} ({mode}): "
            f"error={error_pct:+.2f}% <= {args.max_cruise_error_pct:.2f}%, "
            f"CV={cv_pct:.2f}% <= {args.max_cruise_cv_pct:.2f}%"
        )
        if not passed:
            failures.append(stage)

    if failures and args.fail_on_speed_error:
        raise SystemExit(
            "Cruise-speed acceptance failed for: " + ", ".join(failures)
        )


if __name__ == "__main__":
    main()
