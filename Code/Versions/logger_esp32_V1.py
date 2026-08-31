import csv
import struct
from pathlib import Path
from datetime import datetime

import serial

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM3"
BAUD = 921600

DATA_DIR = Path(r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\raw")
DATA_DIR.mkdir(parents=True, exist_ok=True)

NUM_SENSORS = 3
ACCEL_SCALE = 16384.0  # ±2g
GYRO_SCALE = 131.0     # ±250 deg/s

SYNC = b"\xAA\x55"
PACKET_TYPE_DATA = 0x01
PAYLOAD_SIZE = 4 + 4 + (NUM_SENSORS * 6 * 2)
FRAME_SIZE = 2 + 1 + 1 + PAYLOAD_SIZE + 1

# ============================================================
# CREATE OUTPUT FILE
# ============================================================

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
    print("\nWaiting for ESP32 startup/calibration messages...\n")

    while True:
        line = ser.readline()

        if not line:
            continue

        text = line.decode("utf-8", errors="ignore").strip()

        if text:
            print(text)

        if text == "START_BINARY":
            print("\nBinary stream detected.")
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
    # Search for sync bytes. This lets the logger recover if it starts
    # in the middle of a binary frame or if one frame is corrupted.
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

    if packet_type != PACKET_TYPE_DATA or payload_size != PAYLOAD_SIZE:
        return None

    payload = read_exactly(ser, PAYLOAD_SIZE)
    received_checksum = read_exactly(ser, 1)[0]

    frame_without_checksum = SYNC + header_rest + payload
    expected_checksum = calculate_checksum(frame_without_checksum)

    if received_checksum != expected_checksum:
        return None

    return payload


def decode_payload(payload: bytes):
    offset = 0

    t_us, sample_index = struct.unpack_from("<II", payload, offset)
    offset += 8

    row = {
        "t_us": t_us,
        "sample_index": sample_index,
    }

    # Also keep legacy sensor-1 column names so older analysis habits still work.
    legacy_values = {}

    for sensor_number in range(1, NUM_SENSORS + 1):
        raw_values = struct.unpack_from("<hhhhhh", payload, offset)
        offset += 12

        ax_g = raw_values[0] / ACCEL_SCALE
        ay_g = raw_values[1] / ACCEL_SCALE
        az_g = raw_values[2] / ACCEL_SCALE
        gx_dps = raw_values[3] / GYRO_SCALE
        gy_dps = raw_values[4] / GYRO_SCALE
        gz_dps = raw_values[5] / GYRO_SCALE
        a_resultant_g = (ax_g**2 + ay_g**2 + az_g**2) ** 0.5

        prefix = f"s{sensor_number}"

        row[f"{prefix}_ax_g"] = ax_g
        row[f"{prefix}_ay_g"] = ay_g
        row[f"{prefix}_az_g"] = az_g
        row[f"{prefix}_gx_dps"] = gx_dps
        row[f"{prefix}_gy_dps"] = gy_dps
        row[f"{prefix}_gz_dps"] = gz_dps
        row[f"{prefix}_a_resultant_g"] = a_resultant_g

        if sensor_number == 1:
            legacy_values = {
                "ax_g": ax_g,
                "ay_g": ay_g,
                "az_g": az_g,
                "gx_dps": gx_dps,
                "gy_dps": gy_dps,
                "gz_dps": gz_dps,
                "a_resultant_g": a_resultant_g,
            }

    row.update(legacy_values)
    return row


fieldnames = ["t_us", "sample_index"]

for sensor_number in range(1, NUM_SENSORS + 1):
    prefix = f"s{sensor_number}"
    fieldnames += [
        f"{prefix}_ax_g",
        f"{prefix}_ay_g",
        f"{prefix}_az_g",
        f"{prefix}_gx_dps",
        f"{prefix}_gy_dps",
        f"{prefix}_gz_dps",
        f"{prefix}_a_resultant_g",
    ]

fieldnames += [
    "ax_g",
    "ay_g",
    "az_g",
    "gx_dps",
    "gy_dps",
    "gz_dps",
    "a_resultant_g",
]

# ============================================================
# STARTUP MESSAGE
# ============================================================

print("\n========================================")
print(" ESP32 3-MPU6050 BINARY VIBRATION LOGGER")
print("========================================")
print(f"\nPort: {PORT}")
print(f"Baud: {BAUD}")
print(f"Saving to:\n{output_file}")

input("\nPress ENTER to START recording...")

print("\nOpening serial port...")
print("Press CTRL + C to stop and save the file.\n")

valid_frames = 0
bad_frames = 0

try:
    with serial.Serial(PORT, BAUD, timeout=2) as ser, open(output_file, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        read_until_binary_start(ser)

        print("\nRecording binary data and saving decoded CSV...")
        print("Press CTRL + C to stop.\n")

        while True:
            payload = read_frame(ser)

            if payload is None:
                bad_frames += 1
                continue

            row = decode_payload(payload)
            writer.writerow(row)
            valid_frames += 1

            if valid_frames % 250 == 0:
                print(
                    f"Samples: {valid_frames} | "
                    f"t = {row['t_us'] / 1_000_000.0:.3f} s | "
                    f"bad frames: {bad_frames}",
                    end="\r",
                )

except KeyboardInterrupt:
    print("\n\nRecording stopped by user.")
    print(f"Valid samples saved: {valid_frames}")
    print(f"Bad/corrupted frames skipped: {bad_frames}")
    print(f"\nData saved to:\n{output_file}")
    print("\nDone.")
