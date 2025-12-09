#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""
Configurador ZED-F9P para Moving-Base RTK (MB-RTK) con autodetección.

Arquitectura MB-RTK correcta (según Integration Manual F9P):
- MOVING BASE (MB): emite RTCM (4072.0 + MSM + 1230) por UART2 TX hacia el Rover.
                    Opcionalmente recibe NTRIP por USB para mejorar su posición absoluta.
- ROVER (RV):       recibe RTCM por UART2 RX desde la MB.
                    Publica UBX-NAV-RELPOSNED (heading/baseline) por USB.
                    NO recibe NTRIP directo; solo correcciones de la MB.

Autodetección: escanea /dev/ttyACM0 y /dev/ttyACM1.
               Asigna MB al primero que responde, RV al segundo.

Requiere ubxtool >= 3.2 instalado y en PATH.
"""

from __future__ import annotations
import sys
import time
import argparse
import platform
import os
from dataclasses import dataclass

import subprocess


# ------------------------------------------------------------
# PARÁMETROS
# ------------------------------------------------------------
CANDIDATE_PORTS = ("/dev/ttyACM0", "/dev/ttyACM1")
UART2_BAUD = 460800  # Enlace MB ↔ Rover
ROVER_UART1_BAUD = 115200

# ------------------------------------------------------------

def log(msg: str):
    timestamp = time.strftime("%H:%M:%S")
    print(f"[{timestamp}] {msg}")

def dbg(msg: str):
    pass

# ------------------------------------------------------------

def port_responds(port: str) -> bool:
    """Check if ubxtool can talk to the port, con debug."""
    try:
        cmd = ["ubxtool", "-f", port, "-P", "27", "-p", "MON-VER"]
        log(f"[DEBUG] Probing: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, timeout=10)
        log(f"[DEBUG] Return code: {result.returncode}")
        log(f"[DEBUG] STDOUT: {result.stdout.decode(errors='ignore').strip()}")
        log(f"[DEBUG] STDERR: {result.stderr.decode(errors='ignore').strip()}")
        return b"SW VERSION" in result.stdout or b"SW VERSION" in result.stderr or result.returncode == 0
    except Exception as e:
        log(f"[DEBUG] Exception: {e}")
        return False

def await_ack(ser: Serial, expect_class: int | None = None, expect_id: int | None = None, timeout_s: float = 2.5) -> bool | None:
    for _, parsed in read_ubx_until(ser, timeout_s=timeout_s):
        if not parsed:
            continue
        ident = getattr(parsed, "identity", "")
        if ident in ("ACK-ACK", "ACK-NAK"):
            cls_match = expect_class is None or getattr(parsed, "clsID", None) == expect_class
            id_match  = expect_id   is None or getattr(parsed, "msgID", None) == expect_id
            if not (cls_match and id_match):
                continue
            if ident == "ACK-ACK":
                dbg(f"ACK-ACK (cls=0x{expect_class:02X}, id=0x{expect_id:02X})")
                return True
            else:
                dbg(f"ACK-NAK (cls=0x{expect_class:02X}, id=0x{expect_id:02X})")
                return False
    return None

def send_and_wait_ack(ser: Serial, msg: UBXMessage, wait_s: float = 0.25) -> bool | None:
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    raw = msg.serialize()
    log_hex(f"TX {msg.identity or 'UBX-SET'}", raw)
    ser.write(raw); ser.flush()
    time.sleep(wait_s)
    expect_class = getattr(msg, "_ubxClass", None)
    expect_id = getattr(msg, "_ubxID", None)
    ack = await_ack(ser, expect_class, expect_id, timeout_s=2.5)
    if ack is True:
        log("  → ACK-ACK")
    elif ack is False:
        log("  → ACK-NAK")
    else:
        log("  → (sin ACK)")
    return ack

# ------------------------------------------------------------
# “ARREGLO A CIEGAS” USB UBX (CFG-PRT portID=3)
# ------------------------------------------------------------
def blind_enable_usb_ubx(ser: Serial):
    """
    Intenta habilitar UBX en USB (portID=3) aunque no veamos ACK.
    Habilita in/out UBX y también deja NMEA habilitado (útil para diagnóstico).
    """
    log("Intentando habilitar UBX en USB (CFG-PRT portID=3) a ciegas…")
    try:
        ser.reset_input_buffer()
    except Exception:
        pass
    msg = UBXMessage(
        "CFG", "CFG-PRT",
        SET,
        portID=3,        # USB
        mode=0x08D0,     # 8N1 (no aplica a USB pero es inocuo)
        inProtoMask=0x01 | 0x02,   # UBX + NMEA IN
        outProtoMask=0x01 | 0x02,  # UBX + NMEA OUT
        txReady=0
    )
    raw = msg.serialize()
    log_hex("TX CFG-PRT(USB)", raw)
    ser.write(raw); ser.flush()
    time.sleep(0.5)
    # Leer y descartar cualquier respuesta (ACK, NACK, o basura NMEA)
    # para limpiar el buffer antes del siguiente comando
    try:
        while ser.in_waiting > 0:
            ser.read(ser.in_waiting)
            time.sleep(0.05)
    except Exception:
        pass
    # Puede no haber ACK si UBX OUT está deshabilitado; no lo tratamos como error.

# ------------------------------------------------------------
# SNIFFER de texto/bytes (diagnóstico)
# ------------------------------------------------------------
def sniff_port(port: str, seconds: float = 2.0, max_bytes: int = 512):
    if not LOG.sniff:
        return
    log(f"[SNIFF] Leyendo {seconds:.1f}s de {port} …")
    try:
        ser = open_gnss(port)
    except Exception as e:
        log(f"[SNIFF] no se pudo abrir {port}: {e}")
        return
    end = time.time() + seconds
    buf = b""
    while time.time() < end and len(buf) < max_bytes:
        b = ser.read(64)
        if not b:
            continue
        buf += b
    ser.close()
    if not buf:
        log("[SNIFF] (sin datos)")
    else:
        # mostrar líneas imprimibles y hex preview
        try:
            text = buf.decode("ascii", errors="ignore")
        except Exception:
            text = ""
        preview = text.strip().splitlines()
        if preview:
            log("[SNIFF] Texto detectado (primeras líneas):")
            for line in preview[:10]:
                log(f"        {line[:120]}")
        log(f"[SNIFF] HEX {min(len(buf),max_bytes)} bytes: {hx(buf[:max_bytes])}")

# ------------------------------------------------------------
# CHEQUEO / AUTODETECCIÓN
# ------------------------------------------------------------
COMMON_BAUDS = [9600, 38400, 115200, 57600, 19200, 460800, 921600]

def check_connection_once(port: str, try_fix_usb: bool = True, try_autobaud: bool = True) -> tuple[bool, str | None]:
    log(f"\nChequeando receptor en {port}")
    
    # Primero intentar con el baud por defecto
    bauds_to_try = [USB_BAUD]
    if try_autobaud:
        # Agregar otros bauds comunes
        bauds_to_try.extend([b for b in COMMON_BAUDS if b != USB_BAUD])
    
    for baud in bauds_to_try:
        if baud != USB_BAUD:
            log(f"Probando {baud} bps...")
        try:
            ser = open_gnss(port, baud=baud)
        except Exception as e:
            log(f"No se pudo abrir {port}: {e}")
            continue

        ver = poll_mon_ver(ser, timeout_s=0.8)
        if ver:
            ser.close()
            log(f"OK a {baud} bps: {ver}")
            return True, ver

        # Intento de recuperación: habilitar UBX en USB y reintentar (solo en primer baud)
        if try_fix_usb and baud == USB_BAUD:
            blind_enable_usb_ubx(ser)
            time.sleep(0.3)
            ver = poll_mon_ver(ser, timeout_s=0.8)
            if ver:
                ser.close()
                log(f"OK a {baud} bps tras habilitar USB UBX: {ver}")
                return True, ver

        ser.close()
    
    log(f"No responde en ningún baud rate probado")
    return False, None

def check_connection(port: str, retries: int = 2, sniff_before: bool = True) -> tuple[bool, str | None]:
    if sniff_before:
        sniff_port(port, seconds=1.5, max_bytes=512)
    backoff = 0.4
    for attempt in range(1, retries + 2):  # p.ej. 1 intento + 2 reintentos = 3 totales
        ok, ver = check_connection_once(port, try_fix_usb=True)
        if ok:
            return True, ver
        dbg(f"Reintento {attempt}/{retries+1} en {backoff:.1f}s…")
        time.sleep(backoff)
        backoff *= 1.5
    return False, None


def autodetect_ports() -> tuple[str, str]:
    responding = []
    for p in CANDIDATE_PORTS:
        if port_responds(p):
            responding.append(p)
    if len(responding) < 2:
        log(f"\nError: Solo se detectaron {len(responding)} ZED-F9P respondiendo (se necesitan 2).")
        log(f"Puertos detectados: {responding if responding else 'ninguno'}")
        log("Posibles causas:")
        log("  - Dispositivos no conectados o sin alimentación")
        log("  - gpsd bloqueando los puertos: sudo systemctl stop gpsd.socket gpsd")
        log("  - ModemManager interfiriendo: sudo systemctl stop ModemManager")
        log("  - ubxtool no está en PATH o no funciona correctamente")
        log("  - Timeout insuficiente o problemas de comunicación")
        sys.exit(1)
    mb_port, rv_port = responding[0], responding[1]
    log(f"\nAsignación automática: MB (moving base)={mb_port}  RV (rover)={rv_port}")
    return mb_port, rv_port

# ------------------------------------------------------------

def run_ubxtool(args, port):
    cmd = ["ubxtool", "-f", port, "-P", "27"] + args
    log(f"[DEBUG] Ejecutando: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, timeout=10)
    log(f"[DEBUG] Return code: {result.returncode}")
    log(f"[DEBUG] STDOUT: {result.stdout.decode(errors='ignore').strip()}")
    log(f"[DEBUG] STDERR: {result.stderr.decode(errors='ignore').strip()}")


def configure_moving_base(port: str):
    """
    Configura la Moving Base (MB):
    - TMODE3 desactivado (móvil).
    - Recibe NTRIP (RTCM) por USB para mejorar posición absoluta.
    - Emite RTCM (4072.0 + MSM + 1230) por UART2 TX hacia el Rover.
    - Publica UBX-NAV-PVT por USB.
    """
    log(f"\n=== Configurando MOVING BASE en {port} ===")
    
    # Verificar que el puerto responde antes de configurar
    log("Verificando comunicación...")
    run_ubxtool(["-p", "MON-VER"], port)
    
    # TMODE3 desactivado (modo móvil, no survey-in ni fixed)
    run_ubxtool(["-z", "CFG-TMODE-MODE,0"], port)
    
    # Tasa de navegación/medición a 5Hz (200ms)
    run_ubxtool(["-z", "CFG-RATE-MEAS,200"], port)
    run_ubxtool(["-z", "CFG-RATE-NAV,1"], port)
    
    # UART2: baudrate 460800, RTCM OUT hacia Rover (no recibe RTCM por UART2)
    run_ubxtool(["-z", f"CFG-UART2-BAUDRATE,{UART2_BAUD}"], port)
    run_ubxtool(["-z", "CFG-UART2OUTPROT-RTCM3X,1"], port)
    run_ubxtool(["-z", "CFG-UART2INPROT-RTCM3X,0"], port)
    
    # USB: entrada RTCM (NTRIP), salida UBX (sin NMEA)
    run_ubxtool(["-z", "CFG-USBINPROT-RTCM3X,1"], port)
    run_ubxtool(["-z", "CFG-USBOUTPROT-UBX,1"], port)
    run_ubxtool(["-z", "CFG-USBOUTPROT-NMEA,0"], port)

    # Deshabilitar NMEA por USB
    for msg in [
        "CFG-MSGOUT-NMEA_ID_GGA_USB", "CFG-MSGOUT-NMEA_ID_RMC_USB",
        "CFG-MSGOUT-NMEA_ID_GSA_USB", "CFG-MSGOUT-NMEA_ID_VTG_USB",
        "CFG-MSGOUT-NMEA_ID_GSV_USB", "CFG-MSGOUT-NMEA_ID_GLL_USB",
        "CFG-MSGOUT-NMEA_ID_GST_USB", "CFG-MSGOUT-NMEA_ID_ZDA_USB",
    ]:
        run_ubxtool(["-z", f"{msg},0"], port)
    
    # Habilitar UBX-NAV-PVT por USB (para NTRIP NMEA y monitoreo)
    run_ubxtool(["-z", "CFG-MSGOUT-UBX_NAV_PVT_USB,1"], port)
    
    # --- RTCM por UART2 hacia el Rover ---
    # RTCM 4072.0: Reference station PVT (requerido cada época para MB-RTK)
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART2,1"], port)
    
    # RTCM MSM4 (más liviano que MSM7, suficiente para baseline corta)
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1074_UART2,1"], port)  # GPS MSM4
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1084_UART2,1"], port)  # GLONASS MSM4
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1094_UART2,1"], port)  # Galileo MSM4
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1124_UART2,1"], port)  # BeiDou MSM4
    
    # RTCM 1230: GLONASS code-phase biases (requerido si GLONASS activo)
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1230_UART2,1"], port)
    
    # Guardar configuración
    run_ubxtool(["-p", "SAVE"], port)
    log("Moving Base configurada correctamente")


def configure_pps_only(port: str, polarity: int = 0):
    """
    Configura solo el PPS (Time Pulse) en el receptor.
    - polarity: 0 = flanco de subida, 1 = flanco invertido
    """
    log(f"\n=== Configurando PPS en {port} ===")
    
    # Verificar que el puerto responde
    log("Verificando comunicación...")
    run_ubxtool(["-p", "MON-VER"], port)
    
    # Configurar PPS
    run_ubxtool(["-z", "CFG-TP-TP1_ENA,1"], port)
    run_ubxtool(["-z", f"CFG-TP-POL_TP1,{polarity}"], port)
    
    # Guardar configuración
    run_ubxtool(["-p", "SAVE"], port)
    log(f"PPS configurado correctamente (polarity={polarity})")


def configure_rover(port: str):
    """
    Configura el Rover (RV):
    - TMODE3 desactivado (móvil).
    - Recibe RTCM por UART2 RX desde la Moving Base.
    - Publica UBX-NAV-PVT y UBX-NAV-RELPOSNED (heading/baseline) por USB.
    - NO recibe NTRIP directo; solo correcciones de la MB.
    """
    log(f"\n=== Configurando ROVER en {port} ===")
    
    # Verificar que el puerto responde antes de configurar
    log("Verificando comunicación...")
    run_ubxtool(["-p", "MON-VER"], port)
    
    # TMODE3 desactivado (modo móvil)
    run_ubxtool(["-z", "CFG-TMODE-MODE,0"], port)
    
    # Tasa de navegación/medición a 5Hz (200ms) - debe coincidir con MB
    run_ubxtool(["-z", "CFG-RATE-MEAS,200"], port)
    run_ubxtool(["-z", "CFG-RATE-NAV,1"], port)
    
    # UART2: baudrate 460800, RTCM IN desde MB (no emite RTCM)
    run_ubxtool(["-z", f"CFG-UART2-BAUDRATE,{UART2_BAUD}"], port)
    run_ubxtool(["-z", "CFG-UART2INPROT-RTCM3X,1"], port)
    run_ubxtool(["-z", "CFG-UART2OUTPROT-RTCM3X,0"], port)
    
    # UART1: deshabilitado para limpiar salida
    run_ubxtool(["-z", f"CFG-UART1-BAUDRATE,{ROVER_UART1_BAUD}"], port)
    run_ubxtool(["-z", "CFG-UART1OUTPROT-NMEA,0"], port)
    for msg in [
        "CFG-MSGOUT-NMEA_ID_VTG_UART1", "CFG-MSGOUT-NMEA_ID_GGA_UART1",
        "CFG-MSGOUT-NMEA_ID_RMC_UART1", "CFG-MSGOUT-NMEA_ID_GSA_UART1",
        "CFG-MSGOUT-NMEA_ID_GSV_UART1", "CFG-MSGOUT-NMEA_ID_GLL_UART1",
        "CFG-MSGOUT-NMEA_ID_GST_UART1", "CFG-MSGOUT-NMEA_ID_ZDA_UART1",
    ]:
        run_ubxtool(["-z", f"{msg},0"], port)

    # USB: salida UBX (sin NMEA), sin entrada RTCM (viene por UART2)
    run_ubxtool(["-z", "CFG-USBINPROT-RTCM3X,0"], port)
    run_ubxtool(["-z", "CFG-USBOUTPROT-UBX,1"], port)
    run_ubxtool(["-z", "CFG-USBOUTPROT-NMEA,0"], port)
    for msg in [
        "CFG-MSGOUT-NMEA_ID_GGA_USB", "CFG-MSGOUT-NMEA_ID_RMC_USB",
        "CFG-MSGOUT-NMEA_ID_GSA_USB", "CFG-MSGOUT-NMEA_ID_VTG_USB",
        "CFG-MSGOUT-NMEA_ID_GSV_USB", "CFG-MSGOUT-NMEA_ID_GLL_USB",
        "CFG-MSGOUT-NMEA_ID_GST_USB", "CFG-MSGOUT-NMEA_ID_ZDA_USB",
    ]:
        run_ubxtool(["-z", f"{msg},0"], port)
    
    # Habilitar UBX-NAV-PVT por USB
    run_ubxtool(["-z", "CFG-MSGOUT-UBX_NAV_PVT_USB,1"], port)
    
    # Habilitar UBX-NAV-RELPOSNED por USB (heading y baseline)
    run_ubxtool(["-z", "CFG-MSGOUT-UBX_NAV_RELPOSNED_USB,1"], port)
    
    # PPS normal (polarity=0) y habilitar TP1
    run_ubxtool(["-z", "CFG-TP-TP1_ENA,1"], port)
    run_ubxtool(["-z", "CFG-TP-POL_TP1,0"], port)
    
    # Guardar configuración
    run_ubxtool(["-p", "SAVE"], port)
    log("Rover configurado correctamente")

# ------------------------------------------------------------
# BANNER VERSIONES
# ------------------------------------------------------------

def print_versions():
    log("=== ENTORNO / VERSIONES ===")
    log(f"Python:         {platform.python_version()} ({sys.executable})")
    log(f"SO:             {platform.system()} {platform.release()}  ({platform.platform()})")
    log(f"ubxtool:        (debe estar en PATH)")
    log(f"USER:           {os.environ.get('USER') or os.environ.get('USERNAME')}")
    log(f"PWD:            {os.getcwd()}")
    log("============================\n")

# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(
        description="Configurador ZED-F9P para Moving-Base RTK",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Ejemplos:
  # Configurar ambos receptores (autodetección):
  python configure_f9p_ubxtool.py
  
  # Configurar solo Moving Base en puerto específico:
  python configure_f9p_ubxtool.py --mb-only --port /dev/ttyACM0
  
  # Configurar solo Rover en puerto específico:
  python configure_f9p_ubxtool.py --rover-only --port /dev/ttyACM1
  
  # Configurar solo PPS en un puerto:
  python configure_f9p_ubxtool.py --pps-only --port /dev/ttyACM1 --pps-polarity 0
        """
    )
    
    parser.add_argument("--mb-only", action="store_true",
                        help="Configurar solo Moving Base")
    parser.add_argument("--rover-only", action="store_true",
                        help="Configurar solo Rover")
    parser.add_argument("--pps-only", action="store_true",
                        help="Configurar solo PPS (Time Pulse)")
    parser.add_argument("--port", type=str,
                        help="Puerto serial específico (ej: /dev/ttyACM0)")
    parser.add_argument("--pps-polarity", type=int, choices=[0, 1], default=0,
                        help="Polaridad del PPS: 0=flanco de subida (default), 1=invertido")
    
    return parser.parse_args()


def main():
    args = parse_args()
    
    # Validar combinaciones de argumentos
    exclusive_modes = sum([args.mb_only, args.rover_only, args.pps_only])
    if exclusive_modes > 1:
        log("Error: Solo puedes especificar uno de: --mb-only, --rover-only, --pps-only")
        sys.exit(1)
    
    if exclusive_modes == 1 and not args.port:
        log("Error: Debes especificar --port cuando usas --mb-only, --rover-only o --pps-only")
        sys.exit(1)
    
    print_versions()
    
    # Modo PPS solo
    if args.pps_only:
        log(f"=== CONFIGURACIÓN PPS ÚNICAMENTE ===")
        configure_pps_only(args.port, polarity=args.pps_polarity)
        log("\nConfiguración PPS finalizada correctamente.")
        return
    
    # Modo Moving Base solo
    if args.mb_only:
        log("=== CONFIGURACIÓN MOVING BASE ÚNICAMENTE ===")
        configure_moving_base(args.port)
        log("\nConfiguración Moving Base finalizada correctamente.")
        return
    
    # Modo Rover solo
    if args.rover_only:
        log("=== CONFIGURACIÓN ROVER ÚNICAMENTE ===")
        configure_rover(args.port)
        log("\nConfiguración Rover finalizada correctamente.")
        return
    
    # Modo por defecto: configurar ambos con autodetección
    log("=== CONFIGURACIÓN MB-RTK ZED-F9P ===")
    log("Arquitectura: Moving Base (MB) → RTCM → Rover (RV)")
    log("  MB: emite RTCM, recibe NTRIP (opcional), publica NAV-PVT")
    log("  RV: recibe RTCM de MB, publica NAV-RELPOSNED (heading/baseline)")
    log("")
    
    MB_PORT, RV_PORT = autodetect_ports()
    configure_moving_base(MB_PORT)
    configure_rover(RV_PORT)
    
    log("\n=== RESUMEN ===")
    log(f"Moving Base: {MB_PORT} (UART2 TX → Rover, USB ← NTRIP)")
    log(f"Rover:       {RV_PORT} (UART2 RX ← MB, USB → RELPOSNED)")
    log("\nConfiguración finalizada correctamente.")

if __name__ == "__main__":
    main()

