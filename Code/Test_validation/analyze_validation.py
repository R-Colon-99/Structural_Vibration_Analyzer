# VALIDATION ANALYSIS - Tests 1-3 only
# Structural Vibration Analyzer
#
# Analyzes a CSV produced by logger_esp32_validation.py (paired with
# main_validation.cpp). Computes timing/jitter statistics and per-sensor
# repeated-sample statistics. Does NOT perform any FFT/frequency-domain
# analysis - see analyze_fft.py for that, once a validated operating rate
# has been established.
#
# IMPORTANT: repeated consecutive raw samples are reported as "repeats,"
# not automatically labeled "stale register reads." Distinguishing a
# genuine stale read (poll faster than the MPU's internal update rate)
# from a legitimate identical reading (e.g., a genuinely static sensor, or
# ADC quantization) requires comparing repeat rates ACROSS static,
# excited, and deliberately-oversampled runs - this script reports the
# numbers for each recording; the comparison and interpretation is left to
# you, run-to-run, as intended by the validation plan.

import argparse
import json
from pathlib import Path
from datetime import datetime

import pandas as pd
import numpy as np

import matplotlib
matplotlib.use("Agg")  # safer when launched from the PC app
import matplotlib.pyplot as plt


# ============================================================
# DEFAULT SETTINGS
# ============================================================

DEFAULT_DATA_DIR = Path(
    r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\validation"
)
DEFAULT_PROCESSED_DIR = Path(
    r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\validation_processed"
)

# A run is flagged for closer inspection if jitter (std/mean dt) exceeds
# this ratio - matches the threshold used elsewhere in this project
# (analyze_fft.py's DEFAULT_JITTER_STD_RATIO_LIMIT) for consistency, not
# because it is uniquely "correct."
JITTER_STD_RATIO_FLAG = 0.01


# ============================================================
# ARGUMENTS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Validation analysis for the acquisition-characterization "
            "firmware/logger (Tests 1-3: throughput, timing/jitter, "
            "repeated-sample detection). No FFT is performed here."
        )
    )

    parser.add_argument(
        "--input",
        required=True,
        help="Path to a CSV produced by logger_esp32_validation.py.",
    )
    parser.add_argument(
        "--meta",
        default=None,
        help=(
            "Path to the matching *.meta.json sidecar. If omitted, the "
            "script looks for a file with the same name as --input but "
            "a .meta.json suffix."
        ),
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_PROCESSED_DIR),
        help="Folder for the summary table and plots.",
    )

    return parser.parse_args()


# ============================================================
# LOADING
# ============================================================

def load_metadata(input_path: Path, meta_arg: str | None) -> dict:
    if meta_arg is not None:
        meta_path = Path(meta_arg)
    else:
        meta_path = input_path.with_suffix(".meta.json")

    if not meta_path.exists():
        print(
            f"WARNING: metadata sidecar not found at {meta_path}. "
            "Proceeding without it - bad-frame count and firmware "
            "configuration details will be unavailable in the report.",
            flush=True,
        )
        return {}

    with open(meta_path, "r") as f:
        return json.load(f)


def discover_sensor_names(df: pd.DataFrame) -> list[str]:
    sensor_names = []

    for column in df.columns:
        if column.endswith("_ax_raw"):
            sensor_names.append(column[: -len("_ax_raw")])

    return sensor_names


def has_instrumentation_columns(df: pd.DataFrame, sensor_names: list[str]) -> bool:
    if not sensor_names:
        return False

    probe = f"{sensor_names[0]}_pca_select_us"
    return probe in df.columns


# ============================================================
# TIMING / JITTER ANALYSIS
# ============================================================

def analyze_timing(df: pd.DataFrame, metadata: dict) -> dict:
    t_us = df["t_us"].to_numpy(dtype=np.float64)
    sample_index = df["sample_index"].to_numpy(dtype=np.int64)

    dt_us = np.diff(t_us)

    total_samples = len(df)
    elapsed_s = (t_us[-1] - t_us[0]) / 1_000_000.0 if total_samples > 1 else 0.0

    mean_dt_us = float(np.mean(dt_us)) if len(dt_us) else float("nan")
    median_dt_us = float(np.median(dt_us)) if len(dt_us) else float("nan")
    std_dt_us = float(np.std(dt_us)) if len(dt_us) else float("nan")
    min_dt_us = float(np.min(dt_us)) if len(dt_us) else float("nan")
    max_dt_us = float(np.max(dt_us)) if len(dt_us) else float("nan")

    mean_rate_hz = 1_000_000.0 / mean_dt_us if mean_dt_us else float("nan")
    median_rate_hz = 1_000_000.0 / median_dt_us if median_dt_us else float("nan")

    jitter_std_ratio = (std_dt_us / mean_dt_us) if mean_dt_us else float("nan")
    percentage_jitter = jitter_std_ratio * 100.0

    if len(dt_us):
        max_deviation_ratio = float(
            np.max(np.abs(dt_us - mean_dt_us)) / mean_dt_us
        )
    else:
        max_deviation_ratio = float("nan")

    rms_jitter_us = (
        float(np.sqrt(np.mean((dt_us - mean_dt_us) ** 2))) if len(dt_us) else float("nan")
    )

    # Expected sample_index should increase by exactly 1 each row. Gaps
    # indicate frames that never arrived/were dropped before reaching the
    # logger (distinct from checksum-failed "bad frames", which never get
    # decoded into a row at all and are counted separately, in metadata).
    index_diffs = np.diff(sample_index)
    missing_index_count = int(np.sum(index_diffs[index_diffs > 1] - 1))
    repeated_index_count = int(np.sum(index_diffs <= 0))

    # Cross-check: firmware-reported cycle_duration_us vs. host-computed
    # dt from consecutive t_us values. These should closely agree; a
    # persistent mismatch would suggest something odd (e.g. serial
    # buffering distorting when frames are read vs. when they were
    # actually generated on-device - though t_us and cycle_duration_us
    # are both device-side timestamps, so this is mainly a consistency
    # check on the firmware's own arithmetic, not a host-timing artifact).
    cycle_duration_us = df["cycle_duration_us"].to_numpy(dtype=np.float64)[1:]
    if len(cycle_duration_us) == len(dt_us) and len(dt_us):
        cross_check_diff_us = cycle_duration_us - dt_us
        cross_check_mean_abs_diff_us = float(np.mean(np.abs(cross_check_diff_us)))
        cross_check_max_abs_diff_us = float(np.max(np.abs(cross_check_diff_us)))
    else:
        cross_check_mean_abs_diff_us = float("nan")
        cross_check_max_abs_diff_us = float("nan")

    missed_deadlines_total = (
        int(df["missed_deadlines_total"].iloc[-1]) if total_samples else 0
    )
    i2c_error_total = int(df["i2c_error_total"].iloc[-1]) if total_samples else 0
    i2c_nack_total = int(df["i2c_nack_total"].iloc[-1]) if total_samples else 0
    i2c_timeout_total = int(df["i2c_timeout_total"].iloc[-1]) if total_samples else 0

    target_rate_hz = (
        int(df["target_poll_rate_hz"].iloc[0]) if total_samples else None
    )

    bad_frames = metadata.get("bad_frames") if metadata else None

    return {
        "total_samples": total_samples,
        "elapsed_s": elapsed_s,
        "target_rate_hz": target_rate_hz,
        "mean_rate_hz": mean_rate_hz,
        "median_rate_hz": median_rate_hz,
        "mean_dt_us": mean_dt_us,
        "median_dt_us": median_dt_us,
        "std_dt_us": std_dt_us,
        "min_dt_us": min_dt_us,
        "max_dt_us": max_dt_us,
        "rms_jitter_us": rms_jitter_us,
        "jitter_std_ratio": jitter_std_ratio,
        "percentage_jitter": percentage_jitter,
        "max_deviation_ratio": max_deviation_ratio,
        "missing_sample_index_count": missing_index_count,
        "repeated_sample_index_count": repeated_index_count,
        "missed_deadlines_total": missed_deadlines_total,
        "i2c_error_total": i2c_error_total,
        "i2c_nack_total": i2c_nack_total,
        "i2c_timeout_total": i2c_timeout_total,
        "bad_frames_from_metadata": bad_frames,
        "cross_check_mean_abs_diff_us": cross_check_mean_abs_diff_us,
        "cross_check_max_abs_diff_us": cross_check_max_abs_diff_us,
        "dt_us_series": dt_us,  # kept for plotting, not printed in the table
    }


# ============================================================
# REPEATED-SAMPLE ANALYSIS (Test 3)
# ============================================================

def analyze_repeats_for_sensor(df: pd.DataFrame, sensor_name: str) -> dict:
    ax = df[f"{sensor_name}_ax_raw"].to_numpy()
    ay = df[f"{sensor_name}_ay_raw"].to_numpy()
    az = df[f"{sensor_name}_az_raw"].to_numpy()

    if len(ax) < 2:
        return {
            "repeated_count": 0,
            "repeated_percentage": 0.0,
            "longest_run": 0,
            "total_consecutive_pairs": 0,
        }

    is_repeat = (ax[1:] == ax[:-1]) & (ay[1:] == ay[:-1]) & (az[1:] == az[:-1])

    repeated_count = int(np.sum(is_repeat))
    total_pairs = len(is_repeat)
    repeated_percentage = 100.0 * repeated_count / total_pairs if total_pairs else 0.0

    # Longest consecutive run of repeats (run of TRUE values in is_repeat,
    # +1 since each TRUE represents a repeat of the sample before it - a
    # run of N consecutive TRUEs means N+1 identical samples in a row).
    longest_run = 0
    current_run = 0
    for repeat in is_repeat:
        if repeat:
            current_run += 1
            longest_run = max(longest_run, current_run)
        else:
            current_run = 0

    longest_run_samples = longest_run + 1 if longest_run > 0 else (1 if len(ax) else 0)

    return {
        "repeated_count": repeated_count,
        "repeated_percentage": repeated_percentage,
        "longest_run": longest_run_samples,
        "total_consecutive_pairs": total_pairs,
    }


def analyze_repeats(df: pd.DataFrame, sensor_names: list[str]) -> dict:
    return {
        sensor_name: analyze_repeats_for_sensor(df, sensor_name)
        for sensor_name in sensor_names
    }


# ============================================================
# I2C SUB-TIMING (only if instrumentation columns are present)
# ============================================================

def analyze_i2c_subtiming(df: pd.DataFrame, sensor_names: list[str]) -> dict | None:
    if not has_instrumentation_columns(df, sensor_names):
        return None

    per_sensor = {}
    total_scan_us_per_row = np.zeros(len(df))

    for sensor_name in sensor_names:
        pca_us = df[f"{sensor_name}_pca_select_us"].to_numpy(dtype=np.float64)
        mpu_us = df[f"{sensor_name}_mpu_read_us"].to_numpy(dtype=np.float64)
        sensor_total_us = pca_us + mpu_us

        per_sensor[sensor_name] = {
            "mean_pca_select_us": float(np.mean(pca_us)),
            "mean_mpu_read_us": float(np.mean(mpu_us)),
            "mean_sensor_total_us": float(np.mean(sensor_total_us)),
            "max_sensor_total_us": float(np.max(sensor_total_us)),
        }

        total_scan_us_per_row += sensor_total_us

    return {
        "per_sensor": per_sensor,
        # "Total complete multi-sensor scan time" - derived here as the
        # sum, per cycle, of every sensor's (pca_select_us + mpu_read_us).
        # This is the pure I2C-bus-time total; it excludes serial
        # transmission time, unlike cycle_duration_us, which includes it.
        "mean_total_scan_us": float(np.mean(total_scan_us_per_row)),
        "max_total_scan_us": float(np.max(total_scan_us_per_row)),
    }


# ============================================================
# REPORTING
# ============================================================

def format_summary_table(timing: dict, repeats: dict, i2c_subtiming: dict | None,
                          metadata: dict) -> str:
    lines = []
    lines.append("=" * 60)
    lines.append("VALIDATION SUMMARY")
    lines.append("=" * 60)

    config = metadata.get("config") if metadata else None
    if config:
        lines.append(
            f"Firmware build: {config.get('build_tag')} "
            f"(id {config.get('build_id')}, protocol v{config.get('protocol_version')})"
        )
        lines.append(
            f"DLPF_CFG={config.get('dlpf_cfg')}  SMPLRT_DIV={config.get('smplrt_div')}  "
            f"acquisition_mode={config.get('acquisition_mode')}"
        )
        lines.append(
            f"I2C timing instrumentation: "
            f"{'ENABLED' if config.get('instrumentation_enabled') else 'disabled'}"
        )
        lines.append(f"Sensors detected: {config.get('sensor_count')}  "
                      f"(channels {config.get('active_channels')})")

    if metadata.get("run_label"):
        lines.append(f"Run label: {metadata['run_label']}")

    lines.append("-" * 60)
    lines.append("TIMING / THROUGHPUT")
    lines.append(f"Total samples:                 {timing['total_samples']}")
    lines.append(f"Elapsed recording time (s):    {timing['elapsed_s']:.3f}")
    lines.append(f"Target poll rate (Hz):         {timing['target_rate_hz']}")
    lines.append(f"Mean actual sampling rate (Hz):{timing['mean_rate_hz']:.2f}")
    lines.append(f"Median actual sampling rate(Hz):{timing['median_rate_hz']:.2f}")
    lines.append(f"Mean dt (us):                  {timing['mean_dt_us']:.2f}")
    lines.append(f"Median dt (us):                {timing['median_dt_us']:.2f}")
    lines.append(f"Std dt (us):                   {timing['std_dt_us']:.2f}")
    lines.append(f"Min dt (us):                   {timing['min_dt_us']:.2f}")
    lines.append(f"Max dt (us):                   {timing['max_dt_us']:.2f}")
    lines.append(f"RMS jitter (us):               {timing['rms_jitter_us']:.2f}")
    lines.append(f"Jitter ratio (std/mean):       {timing['jitter_std_ratio']:.4f}"
                 f"{'  <-- ABOVE flag threshold' if timing['jitter_std_ratio'] > JITTER_STD_RATIO_FLAG else ''}")
    lines.append(f"Percentage jitter:             {timing['percentage_jitter']:.2f}%")
    lines.append(f"Max deviation ratio:           {timing['max_deviation_ratio']:.4f}")
    lines.append(f"Missing sample-index count:    {timing['missing_sample_index_count']}")
    lines.append(f"Repeated sample-index count:   {timing['repeated_sample_index_count']}")
    lines.append(f"Missed deadlines (cumulative): {timing['missed_deadlines_total']}")
    lines.append(f"Bad frames (from metadata):    {timing['bad_frames_from_metadata']}")
    lines.append("-" * 60)
    lines.append("I2C ERROR COUNTERS (cumulative, run-scoped)")
    lines.append(f"Total I2C errors:              {timing['i2c_error_total']}")
    lines.append(f"  of which NACK (bus/wiring):  {timing['i2c_nack_total']}")
    lines.append(f"  of which timeout/short-read: {timing['i2c_timeout_total']}")
    lines.append(
        "  (NACK suggests a genuine bus/wiring fault; timeout/short-read "
        "is more consistent with timing overload, but this classification "
        "is an engineering judgement call, not a certainty.)"
    )
    lines.append("-" * 60)
    lines.append("FIRMWARE/HOST TIMING CROSS-CHECK")
    lines.append(
        f"Mean |cycle_duration_us - dt(t_us)|: "
        f"{timing['cross_check_mean_abs_diff_us']:.2f} us"
    )
    lines.append(
        f"Max  |cycle_duration_us - dt(t_us)|: "
        f"{timing['cross_check_max_abs_diff_us']:.2f} us"
    )

    if i2c_subtiming is not None:
        lines.append("-" * 60)
        lines.append("I2C SUB-TIMING (instrumentation was ENABLED for this run)")
        for sensor_name, stats in i2c_subtiming["per_sensor"].items():
            lines.append(
                f"  {sensor_name}: mean PCA select = {stats['mean_pca_select_us']:.1f} us, "
                f"mean MPU read = {stats['mean_mpu_read_us']:.1f} us, "
                f"mean per-sensor total = {stats['mean_sensor_total_us']:.1f} us, "
                f"max per-sensor total = {stats['max_sensor_total_us']:.1f} us"
            )
        lines.append(
            f"  Mean total multi-sensor scan time (I2C only, excludes "
            f"serial TX): {i2c_subtiming['mean_total_scan_us']:.1f} us"
        )
        lines.append(
            f"  Max total multi-sensor scan time (I2C only):            "
            f"{i2c_subtiming['max_total_scan_us']:.1f} us"
        )

    lines.append("-" * 60)
    lines.append("REPEATED CONSECUTIVE RAW SAMPLES (Test 3)")
    lines.append(
        "These are labeled 'repeats' only. Whether a given run's repeat "
        "rate indicates a stale register read (polling faster than the "
        "MPU6050's ~1 kHz internal update) or a legitimate identical "
        "reading (a genuinely static sensor, or ADC quantization) can "
        "only be judged by COMPARING this run's numbers against a static "
        "baseline, an excited/vibrating run, and a deliberately "
        "oversampled run - not from this run in isolation."
    )
    for sensor_name, stats in repeats.items():
        lines.append(
            f"  {sensor_name}: {stats['repeated_count']} / "
            f"{stats['total_consecutive_pairs']} consecutive pairs repeated "
            f"({stats['repeated_percentage']:.2f}%), "
            f"longest consecutive identical run = {stats['longest_run']} samples"
        )

    lines.append("=" * 60)

    return "\n".join(lines)


# ============================================================
# PLOTS
# ============================================================

def plot_interval_vs_sample(dt_us: np.ndarray, output_path: Path, title_suffix: str):
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(np.arange(len(dt_us)), dt_us, linewidth=0.5)
    ax.set_xlabel("Sample number")
    ax.set_ylabel("Sample interval (us)")
    ax.set_title(f"Sample interval vs. sample number{title_suffix}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_interval_histogram(dt_us: np.ndarray, output_path: Path, title_suffix: str):
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.hist(dt_us, bins=100)
    ax.set_xlabel("Sample interval (us)")
    ax.set_ylabel("Count")
    ax.set_title(f"Sample interval histogram{title_suffix}")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


# ============================================================
# MAIN
# ============================================================

def main():
    args = parse_args()

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(input_path)
    metadata = load_metadata(input_path, args.meta)

    sensor_names = discover_sensor_names(df)

    if not sensor_names:
        raise ValueError(
            "No sensor columns found (expected columns like 's1_ax_raw'). "
            "Is this a valid validation CSV from logger_esp32_validation.py?"
        )

    timing = analyze_timing(df, metadata)
    repeats = analyze_repeats(df, sensor_names)
    i2c_subtiming = analyze_i2c_subtiming(df, sensor_names)

    summary_text = format_summary_table(timing, repeats, i2c_subtiming, metadata)
    print(summary_text, flush=True)

    stem = input_path.stem
    summary_path = output_dir / f"{stem}_summary.txt"
    with open(summary_path, "w") as f:
        f.write(summary_text + "\n")

    run_label = metadata.get("run_label")
    title_suffix = f" ({run_label})" if run_label else ""

    interval_plot_path = output_dir / f"{stem}_interval_vs_sample.png"
    histogram_plot_path = output_dir / f"{stem}_interval_histogram.png"

    plot_interval_vs_sample(timing["dt_us_series"], interval_plot_path, title_suffix)
    plot_interval_histogram(timing["dt_us_series"], histogram_plot_path, title_suffix)

    print(f"\nSummary written to:\n{summary_path}", flush=True)
    print(f"Interval plot written to:\n{interval_plot_path}", flush=True)
    print(f"Histogram plot written to:\n{histogram_plot_path}", flush=True)


if __name__ == "__main__":
    main()
