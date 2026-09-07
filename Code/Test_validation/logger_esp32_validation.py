# VALIDATION LOGGER - for main_validation.cpp only
# Structural Vibration Analyzer
#
# This logger decodes the validation firmware's protocol: accelerometer-
# only data, per-cycle timing diagnostics, and cumulative I2C error
# counters. It does NOT perform FFT analysis - see analyze_validation.py
# for that. It is intentionally separate from the production
# logger_esp32.py, which is untouched and still used for the normal
# accel+gyro+temp firmware.
#
# The logger adapts to whatever sensor count and instrumentation setting
# the CONFIG frame reports at runtime. It does not assume exactly 3
# sensors, and does not assume I2C timing instrumentation is on or off -
# both are read from the firmware's own CONFIG frame.

import argparse
import csv
import json
import struct
import time
from pathlib import Path
from datetime import datetime

import serial

# ============================================================
# DEFAULT SETTINGS
# ============================================================

DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 921600
DEFAULT_DATA_DIR = Path(
    r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\validation"
)

ACCEL_SCALE = 4096.0  # +/-8 g, matches main_validation.cpp's ACCEL_CONFIG

SYNC = b"\xAA\x55"

PACKET_TYPE_DATA = 0x01
PACKET_TYPE_CONFIG = 0x02

BASE_DATA_PAYLOAD_SIZE = 32  # t_us, sample_index, cycle_duration_us,
                              # lateness_us, missed_deadlines_total,
                              # i2c_error_total, i2c_nack_total,
                              # i2c_timeout_total (4 bytes each)
BYTES_PER_SENSOR_ACCEL = 6           # ax, ay, az (int16 each)
BYTES_PER_SENSOR_INSTRUMENTATION = 4  # pca_select_us, mpu_read_us (uint16 each)

CHANNEL_TO_SENSOR_NAME = {
    0: "s1",
    1: "s2",
    2: "s3",
}


# ============================================================
# ARGUMENTS
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Validation logger for main_validation.cpp (accel-only "
            "acquisition characterization). Sensor count and I2C "
            "instrumentation setting are both detected automatically from "
            "the firmware's CONFIG frame."
        )
    )

    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--output", default=None, help="Output CSV path.")
    parser.add_argument("--data-dir", default=str(DEFAULT_DATA_DIR))
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional recording duration in seconds.",
    )
    parser.add_argument(
        "--label",
        default=None,
        help=(
            "Optional short label describing this run's condition, e.g. "
            "'static', 'excited_50hz', 'oversample_stress'. Stored in the "
            "metadata sidecar so Tests 1-3's comparison runs stay "
            "distinguishable later."
        ),
    )
    parser.add_argument(
        "--prompt",
        action="store_true",
        help="Ask for ENTER before recording.",
    )

    return parser.parse_args()


args = parse_args()

PORT = args.port
BAUD = args.baud

DATA_DIR = Path(args.data_dir)
DATA_DIR.mkdir(parents=True, exist_ok=True)

if args.output:
    output_file = Path(args.output)
    output_file.parent.mkdir(parents=True, exist_ok=True)
else:
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    label_part = f"_{args.label}" if args.label else ""
    output_file = DATA_DIR / f"validation{label_part}_{timestamp}.csv"

metadata_file = output_file.with_suffix(".meta.json")


# ============================================================
# HELPER FUNCTIONS (framing logic mirrors logger_esp32.py)
# ============================================================

def calculate_checksum(frame_without_checksum: bytes) -> int:
    checksum = 0
    for byte in frame_without_checksum:
        checksum ^= byte
    return checksum


def read_until_binary_start(ser: serial.Serial) -> None:
    print("\nWaiting for ESP32 startup messages...\n", flush=True)

    while True:
        line = ser.readline()

        if not line:
            continue

        text = line.decode("utf-8", errors="ignore").strip()

        if text:
            print(text, flush=True)

        if text == "START_BINARY":
            print("\nBinary stream detected.", flush=True)
            return


def read_exactly(ser: serial.Serial, number_of_bytes: int) -> bytes:
    data = bytearray()

    while len(data) < number_of_bytes:
        chunk = ser.read(number_of_bytes - len(data))

        if not chunk:
            raise TimeoutError("Serial timeout while reading binary packet.")

        data.extend(chunk)

    return bytes(data)


def read_frame(ser: serial.Serial):
    previous = b""

    while True:
        current = ser.read(1)

        if not current:
            raise TimeoutError("Serial timeout while searching for sync bytes.")

        if previous + current == SYNC:
            break

        previous = current

    header_rest = read_exactly(ser, 2)
    packet_type = header_rest[0]
    payload_size = header_rest[1]

    payload = read_exactly(ser, payload_size)
    received_checksum = read_exactly(ser, 1)[0]

    frame_without_checksum = SYNC + header_rest + payload
    expected_checksum = calculate_checksum(frame_without_checksum)

    if received_checksum != expected_checksum:
        return None

    return packet_type, payload


def decode_config_payload(payload: bytes):
    """
    CONFIG payload layout (see main_validation.cpp header comment):
        protocol_version        [u8]
        build_id                [u32]
        build_tag               [8 bytes ASCII]
        dlpf_cfg                [u8]
        smplrt_div              [u8]
        acquisition_mode        [u8]
        instrumentation_enabled [u8]
        target_poll_rate_hz     [u32]
        sensor_count            [u8]
        active_channels         [u8 x sensor_count]
    """
    fixed_size = 1 + 4 + 8 + 1 + 1 + 1 + 1 + 4 + 1

    if len(payload) < fixed_size:
        raise ValueError("Configuration packet is too short.")

    offset = 0

    protocol_version = payload[offset]
    offset += 1

    (build_id,) = struct.unpack_from("<I", payload, offset)
    offset += 4

    build_tag_bytes = payload[offset : offset + 8]
    build_tag = build_tag_bytes.split(b"\x00", 1)[0].decode("ascii", errors="replace")
    offset += 8

    dlpf_cfg = payload[offset]
    offset += 1

    smplrt_div = payload[offset]
    offset += 1

    acquisition_mode = payload[offset]
    offset += 1

    instrumentation_enabled = bool(payload[offset])
    offset += 1

    (target_poll_rate_hz,) = struct.unpack_from("<I", payload, offset)
    offset += 4

    sensor_count = payload[offset]
    offset += 1

    expected_size = offset + sensor_count

    if len(payload) != expected_size:
        raise ValueError(
            "Invalid configuration payload size. "
            f"Expected {expected_size} bytes, received {len(payload)}."
        )

    active_channels = list(payload[offset : offset + sensor_count])

    if sensor_count < 1:
        raise ValueError("ESP32 reported zero active sensors.")

    if len(set(active_channels)) != len(active_channels):
        raise ValueError("Configuration packet contains duplicate PCA channels.")

    sensor_names = [
        CHANNEL_TO_SENSOR_NAME.get(channel, f"ch{channel}")
        for channel in active_channels
    ]

    config = {
        "protocol_version": protocol_version,
        "build_id": build_id,
        "build_tag": build_tag,
        "dlpf_cfg": dlpf_cfg,
        "smplrt_div": smplrt_div,
        "acquisition_mode": acquisition_mode,
        "instrumentation_enabled": instrumentation_enabled,
        "target_poll_rate_hz": target_poll_rate_hz,
        "sensor_count": sensor_count,
        "active_channels": active_channels,
        "sensor_names": sensor_names,
    }

    return config


def wait_for_configuration(ser: serial.Serial):
    print("Waiting for binary configuration packet...", flush=True)

    bad_frames = 0

    while True:
        frame = read_frame(ser)

        if frame is None:
            bad_frames += 1
            continue

        packet_type, payload = frame

        if packet_type != PACKET_TYPE_CONFIG:
            continue

        config = decode_config_payload(payload)

        print(f"Protocol version: {config['protocol_version']}", flush=True)
        print(
            f"Firmware build: {config['build_tag']} "
            f"(id {config['build_id']})",
            flush=True,
        )
        print(
            f"DLPF_CFG={config['dlpf_cfg']}  SMPLRT_DIV={config['smplrt_div']}  "
            f"acquisition_mode={config['acquisition_mode']}",
            flush=True,
        )
        print(
            f"I2C timing instrumentation: "
            f"{'ENABLED' if config['instrumentation_enabled'] else 'disabled'}",
            flush=True,
        )
        print(f"Target poll rate (Hz): {config['target_poll_rate_hz']}", flush=True)
        print(f"Detected sensors: {config['sensor_count']}", flush=True)
        print(
            "Active PCA channels: "
            + ", ".join(str(channel) for channel in config["active_channels"]),
            flush=True,
        )
        print("Logical sensor names: " + ", ".join(config["sensor_names"]), flush=True)

        return config, bad_frames


def expected_data_payload_size(config) -> int:
    per_sensor = BYTES_PER_SENSOR_ACCEL
    if config["instrumentation_enabled"]:
        per_sensor += BYTES_PER_SENSOR_INSTRUMENTATION

    return BASE_DATA_PAYLOAD_SIZE + config["sensor_count"] * per_sensor


def decode_data_payload(payload: bytes, config):
    sensor_names = config["sensor_names"]
    active_channels = config["active_channels"]
    instrumentation_enabled = config["instrumentation_enabled"]

    expected_size = expected_data_payload_size(config)

    if len(payload) != expected_size:
        raise ValueError(
            "Unexpected data payload size. "
            f"Expected {expected_size}, received {len(payload)}."
        )

    offset = 0

    (
        t_us,
        sample_index,
        cycle_duration_us,
        lateness_us,
        missed_deadlines_total,
        i2c_error_total,
        i2c_nack_total,
        i2c_timeout_total,
    ) = struct.unpack_from("<IIIIIIII", payload, offset)
    offset += 32

    row = {
        "t_us": t_us,
        "sample_index": sample_index,
        "cycle_duration_us": cycle_duration_us,
        "lateness_us": lateness_us,
        "missed_deadlines_total": missed_deadlines_total,
        "i2c_error_total": i2c_error_total,
        "i2c_nack_total": i2c_nack_total,
        "i2c_timeout_total": i2c_timeout_total,
        "target_poll_rate_hz": config["target_poll_rate_hz"],
    }

    # Accelerometer values for ALL sensors are packed together first,
    # followed by instrumentation fields for ALL sensors (if enabled) -
    # matches the firmware's sendDataFrame() field order exactly.
    raw_accel = []

    for channel, sensor_name in zip(active_channels, sensor_names):
        ax_raw, ay_raw, az_raw = struct.unpack_from("<hhh", payload, offset)
        offset += 6
        raw_accel.append((sensor_name, channel, ax_raw, ay_raw, az_raw))

    for sensor_name, channel, ax_raw, ay_raw, az_raw in raw_accel:
        ax_g = ax_raw / ACCEL_SCALE
        ay_g = ay_raw / ACCEL_SCALE
        az_g = az_raw / ACCEL_SCALE

        row[f"{sensor_name}_pca_channel"] = channel
        row[f"{sensor_name}_ax_raw"] = ax_raw
        row[f"{sensor_name}_ay_raw"] = ay_raw
        row[f"{sensor_name}_az_raw"] = az_raw
        row[f"{sensor_name}_ax_g"] = ax_g
        row[f"{sensor_name}_ay_g"] = ay_g
        row[f"{sensor_name}_az_g"] = az_g

    if instrumentation_enabled:
        for sensor_name in sensor_names:
            pca_select_us, mpu_read_us = struct.unpack_from("<HH", payload, offset)
            offset += 4
            row[f"{sensor_name}_pca_select_us"] = pca_select_us
            row[f"{sensor_name}_mpu_read_us"] = mpu_read_us
            # "Total time per sensor" is a simple derived sum, not a wire
            # field - see main_validation.cpp header comment.
            row[f"{sensor_name}_sensor_total_us"] = pca_select_us + mpu_read_us

    return row


def build_fieldnames(config):
    fieldnames = [
        "t_us",
        "sample_index",
        "cycle_duration_us",
        "lateness_us",
        "missed_deadlines_total",
        "i2c_error_total",
        "i2c_nack_total",
        "i2c_timeout_total",
        "target_poll_rate_hz",
    ]

    for sensor_name in config["sensor_names"]:
        fieldnames += [
            f"{sensor_name}_pca_channel",
            f"{sensor_name}_ax_raw",
            f"{sensor_name}_ay_raw",
            f"{sensor_name}_az_raw",
            f"{sensor_name}_ax_g",
            f"{sensor_name}_ay_g",
            f"{sensor_name}_az_g",
        ]

        if config["instrumentation_enabled"]:
            fieldnames += [
                f"{sensor_name}_pca_select_us",
                f"{sensor_name}_mpu_read_us",
                f"{sensor_name}_sensor_total_us",
            ]

    return fieldnames


# ============================================================
# STARTUP
# ============================================================

print("\n========================================", flush=True)
print(" ESP32 ACQUISITION VALIDATION LOGGER", flush=True)
print(" (for main_validation.cpp - accel-only, characterization only)", flush=True)
print("========================================", flush=True)
print(f"\nPort: {PORT}", flush=True)
print(f"Baud: {BAUD}", flush=True)
print(f"Saving to:\n{output_file}", flush=True)
print(f"Metadata sidecar:\n{metadata_file}", flush=True)
if args.label:
    print(f"Run label: {args.label}", flush=True)

if args.prompt:
    input("\nPress ENTER to START recording...")

print("\nOpening serial port...", flush=True)
print("Press CTRL + C to stop and save the file.\n", flush=True)

valid_frames = 0
bad_frames = 0
start_time = None
run_start_wallclock = datetime.now().isoformat(timespec="seconds")
config = None

try:
    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        read_until_binary_start(ser)

        config, config_bad_frames = wait_for_configuration(ser)
        bad_frames += config_bad_frames

        payload_size = expected_data_payload_size(config)
        fieldnames = build_fieldnames(config)

        print(f"Expected data payload size: {payload_size} bytes", flush=True)

        with open(output_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            print("\nRecording binary data and saving decoded CSV...", flush=True)

            start_time = time.monotonic()

            while True:
                if (
                    args.duration is not None
                    and (time.monotonic() - start_time) >= args.duration
                ):
                    print("\nRequested duration reached.", flush=True)
                    break

                frame = read_frame(ser)

                if frame is None:
                    bad_frames += 1
                    continue

                packet_type, payload = frame

                if packet_type == PACKET_TYPE_CONFIG:
                    # Ignore repeated config frames after startup.
                    continue

                if packet_type != PACKET_TYPE_DATA:
                    bad_frames += 1
                    continue

                if len(payload) != payload_size:
                    bad_frames += 1
                    continue

                try:
                    row = decode_data_payload(payload, config)
                except ValueError:
                    bad_frames += 1
                    continue

                writer.writerow(row)
                valid_frames += 1

                if valid_frames % 250 == 0:
                    print(
                        f"Samples: {valid_frames} | "
                        f"t = {row['t_us'] / 1_000_000.0:.3f} s | "
                        f"missed deadlines: {row['missed_deadlines_total']} | "
                        f"I2C errors: {row['i2c_error_total']} | "
                        f"bad frames: {bad_frames}",
                        end="\r",
                        flush=True,
                    )

except KeyboardInterrupt:
    print("\n\nRecording stopped by user.", flush=True)

finally:
    print("\n\nRecording finished.", flush=True)
    print(f"Valid samples saved: {valid_frames}", flush=True)
    print(f"Bad/corrupted frames skipped: {bad_frames}", flush=True)
    print(f"\nData saved to:\n{output_file}", flush=True)

    metadata = {
        "run_label": args.label,
        "run_start_wallclock": run_start_wallclock,
        "run_end_wallclock": datetime.now().isoformat(timespec="seconds"),
        "output_csv": str(output_file),
        "port": PORT,
        "baud": BAUD,
        "valid_frames": valid_frames,
        "bad_frames": bad_frames,
        "config": config,  # None if the run failed before CONFIG was received
        "logger_notes": (
            "Produced by logger_esp32_validation.py, matched to "
            "main_validation.cpp only. Not compatible with the production "
            "logger_esp32.py protocol."
        ),
    }

    with open(metadata_file, "w") as mf:
        json.dump(metadata, mf, indent=2)

    print(f"Metadata saved to:\n{metadata_file}", flush=True)
    print("\nDone.", flush=True)
