# ZED-F9P Moving-Base RTK Configuration

Automated configuration tools for u-blox ZED-F9P dual-receiver Moving-Base RTK architecture. Implements high-precision relative positioning between two mobile GNSS units using RTCM3 corrections and NTRIP services, computing real-time heading and baseline for autonomous navigation and robotics applications.

**Tested:** Ubuntu 20.04 LTS · ROS Noetic · ZED-F9P Firmware 1.51

## Moving-Base RTK Architecture

```
                    ┌─────────────────┐
   NTRIP caster ───►│  Moving Base    │
   (mejor pos abs)  │  /dev/ttyACM0   │
                    │                 │
                    │  UART2 TX ──────┼───► RTCM (4072.0 + MSM4 + 1230)
                    │  USB ◄── NTRIP  │              │
                    │  USB ──► NAV-PVT│              │
                    └─────────────────┘              │
                                                     ▼
                    ┌─────────────────┐              │
                    │     Rover       │◄─────────────┘
                    │  /dev/ttyACM1   │     UART2 RX (RTCM from MB)
                    │                 │
                    │  USB ──► NAV-PVT│
                    │  USB ──► NAV-RELPOSNED (heading/baseline)
                    └─────────────────┘
```

### Receiver Roles

| Receiver | Port | Function |
|----------|--------|---------|
| **Moving Base (MB)** | `/dev/ttyACM0` | Receives NTRIP corrections via USB to improve absolute position. Transmits RTCM corrections via UART2 TX to Rover. Publishes `NAV-PVT`. |
| **Rover (RV)** | `/dev/ttyACM1` | Receives RTCM corrections via UART2 RX from Moving Base. Publishes `NAV-PVT` and `NAV-RELPOSNED` (heading and baseline). |

### Configuration Applied by Script

| Parameter | Moving Base | Rover |
|-----------|-------------|-------|
| TMODE3 | Disabled (mobile) | Disabled (mobile) |
| Navigation Rate | 5 Hz (200 ms) | 5 Hz (200 ms) |
| UART2 Baudrate | 460800 | 460800 |
| UART2 RTCM | OUT (to Rover) | IN (from MB) |
| USB RTCM | IN (NTRIP) | Disabled |
| USB UBX | NAV-PVT | NAV-PVT, NAV-RELPOSNED |
| RTCM Messages | 4072.0, 1074, 1084, 1094, 1124, 1230 | — |

## Requirements

- Access to `/dev/ttyACM0` and `/dev/ttyACM1` for both receivers.
- `ubxtool` >= 3.2 in `PATH` (included in `gpsd-py3` package):
  ```bash
  pip install gpsd-py3
  ```
- Optional: disable services that may block serial ports:
  ```bash
  sudo systemctl stop gpsd.socket gpsd
  sudo systemctl stop ModemManager
  ```
- Python 3 available on the system.

## Quick Start

### 1. Configure Receivers

```bash
cd gps-config
python configure_f9p_ubxtool.py
```

The script will:
1. Auto-detect ports (ACM0 → MB, ACM1 → Rover).
2. Configure both receivers via `ubxtool`.
3. Save configuration to flash memory.

Expected output:
```
[HH:MM:SS] Auto-assignment: MB (moving base)=/dev/ttyACM0  RV (rover)=/dev/ttyACM1
...
[HH:MM:SS] Moving Base: /dev/ttyACM0 (UART2 TX → Rover, USB ← NTRIP)
[HH:MM:SS] Rover:       /dev/ttyACM1 (UART2 RX ← MB, USB → RELPOSNED)
[HH:MM:SS] Configuration completed successfully.
```

### 2. Connect UART2 Hardware

Physical connections:
- **MB UART2 TX** → **Rover UART2 RX**
- (Optional) **MB UART2 RX** ← **Rover UART2 TX** (not currently used)
- Common **GND** between both units

### 3. Launch ROS

```bash
roslaunch gps gps.launch
```

Published topics:
- `/ublox_moving_base/navpvt` — Moving Base position (improved by NTRIP).
- `/ublox_rover/navpvt` — Rover position.
- `/ublox_rover/navheading` — Heading computed from RELPOSNED.
- `/ublox_rover/navrelposned` — Relative position Rover→MB (baseline).

## ROS Configuration Files

| File | Description |
|---------|-------------|
| `gps/config/zed_f9p_primary.yaml` | Moving Base configuration |
| `gps/config/zed_f9p_moving.yaml` | Rover configuration |
| `gps/launch/ntrip.launch` | Launch file with NTRIP + both receivers |
| `gps/config/ekf_ardusimple.yaml` | EKF using Rover topics |

## Work-in-Progress Scripts (Not Recommended)nded)

- `configure_f9p.py`: Direct configuration attempt via `pyubx2`/`pyserial`; in development.
- `configure_f9p.sh`: Initial draft for quick tests with `ubxtool`.

## Troubleshooting

| Issue | Solution |
|----------|----------|
| Script does not detect ports | `sudo systemctl stop gpsd.socket gpsd ModemManager` |
| "HPG Ref nav_rate should be 1 Hz" | Already handled in YAML with `meas_rate`/`nav_rate` |
| NTRIP checksum mismatch | Normal during startup; wait for stabilization |
| No heading/RELPOSNED output | Verify UART2 connection MB→Rover |
| `ubxtool -p MON-VER` does not respond | Disconnect USB cable from problematic receiver, wait 5 seconds, reconnect |
| "Failed to poll MonVER" in ROS | Receiver in corrupted state; apply physical USB reset (disconnect/reconnect) |

## References

- [ZED-F9P Integration Manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
