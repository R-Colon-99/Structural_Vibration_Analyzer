import serial
import csv
from pathlib import Path
from datetime import datetime

# ============================================================
# SETTINGS
# ============================================================

PORT = "COM3"
BAUD = 921600

DATA_DIR = Path(r"F:\RLenovo99\Documents\Projects\Structural_Vibration_Analyzer\Data\raw")
DATA_DIR.mkdir(parents=True, exist_ok=True)

# ============================================================
# CREATE OUTPUT FILE
# ============================================================

timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
output_file = DATA_DIR / f"esp32_vibration_{timestamp}.csv"

# ============================================================
# STARTUP MESSAGE
# ============================================================

print("\n========================================")
print(" ESP32 STRUCTURAL VIBRATION LOGGER")
print("========================================")

print(f"\nPort: {PORT}")
print(f"Baud: {BAUD}")
print(f"Saving to:\n{output_file}")

input("\nPress ENTER to START recording...")

print("\nRecording...")
print("Press CTRL + C to stop and save the file.\n")

# ============================================================
# SERIAL LOGGING
# ============================================================

try:
    with serial.Serial(PORT, BAUD, timeout=2) as ser, open(output_file, "w", newline="") as f:
        writer = csv.writer(f)
        header_written = False

        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()

            if not line:
                continue

            print(line)

            if line.startswith("#"):
                continue

            parts = line.split(",")

            if parts[0] == "t_us":
                writer.writerow(parts)
                header_written = True
                continue

            if header_written and len(parts) == 8:
                writer.writerow(parts)

except KeyboardInterrupt:
    print("\n\nRecording stopped by user.")
    print(f"\nData saved to:\n{output_file}")
    print("\nDone.")