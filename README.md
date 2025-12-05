# Configuración ZED-F9P para Moving-Base RTK (MB-RTK)

Este repo contiene utilidades para configurar dos receptores u-blox ZED-F9P en modo **Moving-Base RTK**, donde ambos receptores están en movimiento y se obtiene posición relativa (heading/baseline) entre ellos.

## Arquitectura MB-RTK

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

### Roles

| Receptor | Puerto | Función |
|----------|--------|---------|
| **Moving Base (MB)** | `/dev/ttyACM0` | Recibe NTRIP por USB (mejora posición absoluta). Emite RTCM por UART2 TX hacia el Rover. Publica `NAV-PVT`. |
| **Rover (RV)** | `/dev/ttyACM1` | Recibe RTCM por UART2 RX desde la MB. Publica `NAV-PVT` y `NAV-RELPOSNED` (heading y baseline). |

### Configuración aplicada por el script

| Parámetro | Moving Base | Rover |
|-----------|-------------|-------|
| TMODE3 | Disabled (móvil) | Disabled (móvil) |
| Tasa navegación | 5 Hz (200 ms) | 5 Hz (200 ms) |
| UART2 baudrate | 460800 | 460800 |
| UART2 RTCM | OUT (hacia Rover) | IN (desde MB) |
| USB RTCM | IN (NTRIP) | Deshabilitado |
| USB UBX | NAV-PVT | NAV-PVT, NAV-RELPOSNED |
| RTCM emitidos | 4072.0, 1074, 1084, 1094, 1124, 1230 | — |

## Requisitos

- Acceso a `/dev/ttyACM0` y `/dev/ttyACM1` para ambos receptores.
- `ubxtool` ≥ 3.2 en el `PATH` (viene con `gpsd-py3`):
  ```bash
  pip install gpsd-py3
  ```
- Opcional: desactivar servicios que puedan tomar los puertos:
  ```bash
  sudo systemctl stop gpsd.socket gpsd
  sudo systemctl stop ModemManager
  ```
- Python 3 disponible en el sistema.

## Uso rápido

### 1. Configurar los receptores

```bash
cd gps-config
python configure_f9p_ubxtool.py
```

El script:
1. Autodetecta los puertos (ACM0 → MB, ACM1 → Rover).
2. Configura ambos receptores vía `ubxtool`.
3. Guarda la configuración en flash.

Salida esperada:
```
[HH:MM:SS] Asignación automática: MB (moving base)=/dev/ttyACM0  RV (rover)=/dev/ttyACM1
...
[HH:MM:SS] Moving Base: /dev/ttyACM0 (UART2 TX → Rover, USB ← NTRIP)
[HH:MM:SS] Rover:       /dev/ttyACM1 (UART2 RX ← MB, USB → RELPOSNED)
[HH:MM:SS] Configuración finalizada correctamente.
```

### 2. Conectar hardware UART2

Conectar físicamente:
- **MB UART2 TX** → **Rover UART2 RX**
- (Opcional) **MB UART2 RX** ← **Rover UART2 TX** (no usado actualmente)
- **GND** común entre ambos

### 3. Lanzar ROS

```bash
roslaunch gps gps.launch
```

Topics publicados:
- `/ublox_moving_base/navpvt` — Posición de la MB (mejorada por NTRIP).
- `/ublox_rover/navpvt` — Posición del Rover.
- `/ublox_rover/navheading` — Heading calculado desde RELPOSNED.
- `/ublox_rover/navrelposned` — Posición relativa Rover→MB (baseline).

## Archivos de configuración ROS

| Archivo | Descripción |
|---------|-------------|
| `gps/config/zed_f9p_primary.yaml` | Config para Moving Base |
| `gps/config/zed_f9p_moving.yaml` | Config para Rover |
| `gps/launch/ntrip.launch` | Launch con NTRIP + ambos receptores |
| `gps/config/ekf_ardusimple.yaml` | EKF usando topics del Rover |

## Scripts en progreso (no recomendados)

- `configure_f9p.py`: intento de configuración directa vía `pyubx2`/`pyserial`; en desarrollo.
- `configure_f9p.sh`: borrador inicial para pruebas con `ubxtool`.

## Troubleshooting

| Problema | Solución |
|----------|----------|
| Script no detecta puertos | `sudo systemctl stop gpsd.socket gpsd ModemManager` |
| "HPG Ref nav_rate should be 1 Hz" | Ya manejado en YAML con `meas_rate`/`nav_rate` |
| NTRIP checksum mismatch | Normal durante arranque; esperar estabilización |
| No hay heading/RELPOSNED | Verificar conexión UART2 MB→Rover |
| `ubxtool -p MON-VER` no responde | Desconectar cable USB del receptor problemático, esperar 5 segundos, reconectar |
| "Failed to poll MonVER" en ROS | Receptor en estado corrupto; aplicar reset físico USB (desconectar/reconectar) |

## Referencias

- [ZED-F9P Integration Manual](https://content.u-blox.com/sites/default/files/ZED-F9P_IntegrationManual_UBX-18010802.pdf)
