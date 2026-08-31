# V5 - App-ready auto-detect binary logger with legacy --num-sensors compatibility
# Structural Vibration Analyzer

import argparse
import csv
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
    r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\raw"
)

ACCEL_SCALE = 4096.0  # +/-8 g
GYRO_SCALE = 131.0    # +/-250 deg/s

SYNC = b"\xAA\x55"

PACKET_TYPE_DATA = 0x01
PACKET_TYPE_CONFIG = 0x02

BYTES_PER_SENSOR = 12
BASE_DATA_PAYLOAD_SIZE = 8

# PCA channel 0 -> s1, channel 1 -> s2, channel 2 -> s3.
# Keeping the logical name tied to the physical PCA channel prevents
# sensor identities from shifting when a middle sensor is disconnected.
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
            "ESP32 binary logger for the Structural Vibration Analyzer. "
            "Sensor count and active PCA channels are detected automatically."
        )
    )

    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port to use. Default: {DEFAULT_PORT}",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate. Default: {DEFAULT_BAUD}",
    )

    parser.add_argument(
        "--output",
        default=None,
        help="Output CSV file path. If omitted, a timestamped file is created.",
    )

    parser.add_argument(
        "--data-dir",
        default=str(DEFAULT_DATA_DIR),
        help="Folder used for timestamped output files when --output is omitted.",
    )

    parser.add_argument(
        "--num-sensors",
        type=int,
        choices=[1, 2, 3],
        default=None,
        help=(
            "Expected sensor count supplied by the PC app. "
            "The ESP32 still auto-detects the actual sensors; this value is "
            "used only to verify that the detected count matches the app selection."
        ),
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional recording duration in seconds. If omitted, records until stopped.",
    )

    parser.add_argument(
        "--prompt",
        action="store_true",
        help="Ask for ENTER before recording. Useful when running manually.",
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
    output_file = DATA_DIR / f"esp32_vibration_{timestamp}.csv"


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def calculate_checksum(frame_without_checksum: bytes) -> int:
    checksum = 0

    for byte in frame_without_checksum:
        checksum ^= byte

    return checksum


def read_until_binary_start(ser: serial.Serial) -> None:
    print("\nWaiting for ESP32 startup/calibration messages...\n", flush=True)

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
    """
    Search for the AA 55 sync bytes, then read a variable-length frame.

    Returns:
        (packet_type, payload) when a valid frame is received.
        None when a frame is malformed or fails checksum.
    """
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
    if len(payload) < 1:
        raise ValueError("Configuration packet is empty.")

    sensor_count = payload[0]
    expected_size = 1 + sensor_count

    if len(payload) != expected_size:
        raise ValueError(
            "Invalid configuration payload size. "
            f"Expected {expected_size} bytes, received {len(payload)}."
        )

    active_channels = list(payload[1:])

    if sensor_count < 1:
        raise ValueError("ESP32 reported zero active sensors.")

    if len(set(active_channels)) != len(active_channels):
        raise ValueError("Configuration packet contains duplicate PCA channels.")

    sensor_names = []

    for channel in active_channels:
        sensor_name = CHANNEL_TO_SENSOR_NAME.get(channel, f"ch{channel}")
        sensor_names.append(sensor_name)

    return sensor_count, active_channels, sensor_names


def wait_for_configuration(ser: serial.Serial):
    print("Waiting for binary sensor configuration packet...", flush=True)

    bad_frames = 0

    while True:
        frame = read_frame(ser)

        if frame is None:
            bad_frames += 1
            continue

        packet_type, payload = frame

        if packet_type != PACKET_TYPE_CONFIG:
            continue

        sensor_count, active_channels, sensor_names = decode_config_payload(payload)

        print(f"Detected sensors: {sensor_count}", flush=True)
        print(
            "Active PCA channels: "
            + ", ".join(str(channel) for channel in active_channels),
            flush=True,
        )
        print(
            "Logical sensor names: " + ", ".join(sensor_names),
            flush=True,
        )

        return sensor_count, active_channels, sensor_names, bad_frames


def decode_data_payload(payload: bytes, active_channels, sensor_names):
    expected_payload_size = (
        BASE_DATA_PAYLOAD_SIZE + len(sensor_names) * BYTES_PER_SENSOR
    )

    if len(payload) != expected_payload_size:
        raise ValueError(
            "Unexpected data payload size. "
            f"Expected {expected_payload_size}, received {len(payload)}."
        )

    offset = 0

    t_us, sample_index = struct.unpack_from("<II", payload, offset)
    offset += 8

    row = {
        "t_us": t_us,
        "sample_index": sample_index,
    }

    for channel, sensor_name in zip(active_channels, sensor_names):
        raw_values = struct.unpack_from("<hhhhhh", payload, offset)
        offset += 12

        ax_g = raw_values[0] / ACCEL_SCALE
        ay_g = raw_values[1] / ACCEL_SCALE
        az_g = raw_values[2] / ACCEL_SCALE

        gx_dps = raw_values[3] / GYRO_SCALE
        gy_dps = raw_values[4] / GYRO_SCALE
        gz_dps = raw_values[5] / GYRO_SCALE

        a_resultant_g = (ax_g**2 + ay_g**2 + az_g**2) ** 0.5

        row[f"{sensor_name}_pca_channel"] = channel
        row[f"{sensor_name}_ax_g"] = ax_g
        row[f"{sensor_name}_ay_g"] = ay_g
        row[f"{sensor_name}_az_g"] = az_g
        row[f"{sensor_name}_gx_dps"] = gx_dps
        row[f"{sensor_name}_gy_dps"] = gy_dps
        row[f"{sensor_name}_gz_dps"] = gz_dps
        row[f"{sensor_name}_a_resultant_g"] = a_resultant_g

    # Preserve legacy single-sensor-style columns when s1 exists.
    if "s1" in sensor_names:
        row["ax_g"] = row["s1_ax_g"]
        row["ay_g"] = row["s1_ay_g"]
        row["az_g"] = row["s1_az_g"]
        row["gx_dps"] = row["s1_gx_dps"]
        row["gy_dps"] = row["s1_gy_dps"]
        row["gz_dps"] = row["s1_gz_dps"]
        row["a_resultant_g"] = row["s1_a_resultant_g"]
    else:
        row["ax_g"] = ""
        row["ay_g"] = ""
        row["az_g"] = ""
        row["gx_dps"] = ""
        row["gy_dps"] = ""
        row["gz_dps"] = ""
        row["a_resultant_g"] = ""

    return row


def build_fieldnames(sensor_names):
    fieldnames = ["t_us", "sample_index"]

    for sensor_name in sensor_names:
        fieldnames += [
            f"{sensor_name}_pca_channel",
            f"{sensor_name}_ax_g",
            f"{sensor_name}_ay_g",
            f"{sensor_name}_az_g",
            f"{sensor_name}_gx_dps",
            f"{sensor_name}_gy_dps",
            f"{sensor_name}_gz_dps",
            f"{sensor_name}_a_resultant_g",
        ]

    # Legacy convenience columns.
    fieldnames += [
        "ax_g",
        "ay_g",
        "az_g",
        "gx_dps",
        "gy_dps",
        "gz_dps",
        "a_resultant_g",
    ]

    return fieldnames


# ============================================================
# STARTUP
# ============================================================

print("\n========================================", flush=True)
print(" ESP32 MPU6050 BINARY VIBRATION LOGGER V4", flush=True)
print("========================================", flush=True)
print(f"\nPort: {PORT}", flush=True)
print(f"Baud: {BAUD}", flush=True)
if args.num_sensors is None:
    print("Sensor count: automatic", flush=True)
else:
    print(
        f"Sensor count selected by app: {args.num_sensors} "
        "(ESP32 detection will be verified)",
        flush=True,
    )
print(f"Saving to:\n{output_file}", flush=True)

if args.prompt:
    input("\nPress ENTER to START recording...")

print("\nOpening serial port...", flush=True)
print(
    "Stop from the PC app or press CTRL + C to stop and save the file.\n",
    flush=True,
)

valid_frames = 0
bad_frames = 0
start_time = None

try:
    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        read_until_binary_start(ser)

        (
            sensor_count,
            active_channels,
            sensor_names,
            config_bad_frames,
        ) = wait_for_configuration(ser)

        bad_frames += config_bad_frames

        if (
            args.num_sensors is not None
            and sensor_count != args.num_sensors
        ):
            raise RuntimeError(
                "Sensor-count mismatch: "
                f"the PC app selected {args.num_sensors} sensor(s), "
                f"but the ESP32 detected {sensor_count} sensor(s) "
                f"on PCA channels {active_channels}. "
                "Check the physical connections or change the app selection."
            )

        expected_data_payload_size = (
            BASE_DATA_PAYLOAD_SIZE + sensor_count * BYTES_PER_SENSOR
        )

        fieldnames = build_fieldnames(sensor_names)

        print(
            f"Expected data payload size: {expected_data_payload_size} bytes",
            flush=True,
        )

        with open(output_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            print(
                "\nRecording binary data and saving decoded CSV...",
                flush=True,
            )

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

                if len(payload) != expected_data_payload_size:
                    bad_frames += 1
                    continue

                try:
                    row = decode_data_payload(
                        payload,
                        active_channels,
                        sensor_names,
                    )
                except ValueError:
                    bad_frames += 1
                    continue

                writer.writerow(row)
                valid_frames += 1

                if valid_frames % 250 == 0:
                    print(
                        f"Samples: {valid_frames} | "
                        f"t = {row['t_us'] / 1_000_000.0:.3f} s | "
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
    print("\nDone.", flush=True)
