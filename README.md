## PID control with LED and temperature (Arduino Nano + Python tools)

### Overview
This project implements a PID‑controlled heating/cooling loop driven by an LED (PWM on `D5`) and a fan (PWM on `D6`), with temperature sensing via an `AD7793` ADC reading a thermocouple/PT100. The firmware runs on an Arduino Nano (`atmega328p`) using PlatformIO. Optional Python scripts provide simple GUIs and utilities for serial interaction and data analysis.

### Hardware (typical)
- **MCU**: Arduino Nano (`atmega328p`)
- **ADC**: `AD7793` on SPI
- **Heater**: LED on `D5` (PWM)
- **Fan**: on `D6` (PWM)
- **Sensors**: Thermocouple/PT100 via `AD7793`

Pins can be changed in `src/main.cpp` (`led`, `fan`).

## Getting started

### 1) Clone and prerequisites
```bash
git clone <your-repo-url>
cd PID-control-with-LED-and-temperature
```
- Install PlatformIO (VS Code extension or CLI). CLI install: `pipx install platformio` or `pip install platformio`.
- Install Python 3.9+.

### 2) Python environment (optional tools)
```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\\Scripts\\activate
pip install -r requirements.txt
```
This provides `pyserial`, `numpy`, `matplotlib`, `scipy`, `pandas`, `scikit-learn`, `opencv-python` for the utilities under `test/`.

### 3) Build and upload firmware (PlatformIO)
The project is configured for Arduino Nano.

Relevant config: see `platformio.ini` (`env:nano`, `board = nanoatmega328`).

Using VS Code + PlatformIO:
- Open the project folder
- Click “Build” then “Upload” (autodetects the serial port)

Using CLI:
```bash
pio run -e nano
pio run -e nano -t upload
```
If the port is not detected, set `upload_port` in `platformio.ini` or pass `--upload-port`.

### 4) Serial monitor
Open a serial monitor at `9600` baud to view logs and send commands.
```bash
pio device monitor -b 9600
```

## Runtime control (serial commands)
The firmware accepts simple newline‑terminated commands over serial:

- `START` — start five‑stage thermocycling (state machine)
- `START_SIMPLE` — start two‑threshold cycling (upper/lower) without PID printouts
- `STOP` — stop outputs (LED/fan off)
- `C<integer>` — set total number of cycles
  - Example: `C30`
- `F<phaseParams>` — configure five‑stage thermocycle phases; semicolon‑separated `temp,holdSeconds`
  - Phases (in order): initial denaturation, stage 2, stage 3, stage 4, final extension
  - Example: `F95,15;55,30;72,30;72,30;72,60;`
- `T<upper>,<lower>,<setpoint>` — configure two‑threshold cycling and PID setpoint (for simple cycling)
  - Example: `T95,60,95`

Notes:
- Temperature readings and PID debug information print to serial.
- The LED intensity (heater) and fan speed are modulated via PWM on `D5` and `D6`.

## Optional Python utilities
Utilities in `test/` can visualize data or provide simple GUIs.

- Launch an example GUI (one of these, as needed):
```bash
python3 test/GUI3.py
```

These scripts rely on `tkinter` (standard library on many Python installs) and `matplotlib`. Adjust serial port inside the scripts if needed.

## Repository layout
- `src/` — Arduino firmware (`main.cpp` contains PID, thermocycling logic, serial commands)
- `lib/TemperatureControl/` — device/ADC config
- `test/` — Python scripts (GUIs, analysis)
- `requirements.txt` — Python dependencies for optional tools
- `platformio.ini` — PlatformIO board and build settings

## Data and large files
Image files, `.txt` data, and spreadsheets are ignored by Git (see `.gitignore`). Place your experimental data under `Data/` or `test/Data/` locally—they will not be uploaded.

## Troubleshooting
- If upload fails, set the correct serial port via `upload_port` in `platformio.ini` or use the PlatformIO UI to select the port.
- If serial output is garbled, confirm baud `9600` on both ends.
- If temperature does not change, verify sensor wiring and `AD7793` SPI connections, and confirm PWM pins match your hardware.

## References
- PlatformIO: [Documentation](https://docs.platformio.org/)
- Arduino Nano (ATmega328P): [Board info](https://store.arduino.cc/)

