# ESP32 Seeed Drone

## Project Overview

This project refactors a cheap drone bought on AliExpress with the goal of making it autonomous. The original drone is composed of two boards:

- **Flight control board**: handles drone flight mechanics (kept as-is)
- **Brain board**: ESP32-based, provides WiFi connectivity, video camera, and sends commands to the flight control board (this is the board being replaced)

## Hardware Replacement

The original brain board is being replaced with:

- **Seeed Studio ESP32-S3** (with video camera support)
- **ICM-20948 IMU** (9-axis motion tracking)

## Development Environment

- **Framework**: ESP-IDF (Espressif IoT Development Framework)
- Always ensure the ESP-IDF environment is active before working on the project. Verify with:
  ```bash
  idf.py --version
  ```
- To detect a connected ESP32 device and identify its version:
  ```bash
  python -m esptool --port /dev/ttyACM0 flash_id
  ```
- **Device port**: `/dev/ttyACM0`
- **Clang LSP**: `clangd`

## Workflow Rules

- **Commits**: When a feature is complete and validated by the user, create a commit for that feature. Do not commit unvalidated work.
- **Environment check**: Confirm `idf.py` is available before running build/flash commands.
