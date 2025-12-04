#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""
Configurador ZED-F9P (PR/MB) con autodetección (ACM0/ACM1) y configuración vía ubxtool.

- Autodetección: escanea /dev/ttyACM0 y /dev/ttyACM1, asigna PR al primero que responde y MB al segundo.
- Configura PR para recibir NTRIP (RTCM) por USB y enviar RTCM por UART2 TX a MB.
- Configura MB para recibir RTCM por UART2 RX, enviar NMEA+UBX por USB, PPS invertido, y transmitir VTG por UART1 TX a 1Hz, navegación a 5Hz.
- Requiere ubxtool >= 3.2 instalado y en PATH.
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
MB_UART1_BAUD = 115200

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
    pr_port, mb_port = responding[0], responding[1]
    log(f"\nAsignación automática: PR={pr_port}  MB={mb_port}")
    return pr_port, mb_port

# ------------------------------------------------------------

def run_ubxtool(args, port):
    cmd = ["ubxtool", "-f", port, "-P", "27"] + args
    log(f"[DEBUG] Ejecutando: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=True, timeout=10)
    log(f"[DEBUG] Return code: {result.returncode}")
    log(f"[DEBUG] STDOUT: {result.stdout.decode(errors='ignore').strip()}")
    log(f"[DEBUG] STDERR: {result.stderr.decode(errors='ignore').strip()}")

def configure_pr(port: str):
    log(f"\nConfigurando PR en {port}")
    
    # Configurar tasa de navegación y medición a 5Hz (200ms)
    run_ubxtool(["-z", "CFG-RATE-MEAS,200"], port)  # medición cada 200ms = 5Hz
    run_ubxtool(["-z", "CFG-RATE-NAV,1"], port)     # 1 solución por medición
    
    # UART2: baudrate 460800, protocolo RTCM out
    run_ubxtool(["-z", "CFG-UART2-BAUDRATE,460800"], port)
    run_ubxtool(["-z", "CFG-UART2OUTPROT-RTCM3X,1"], port)
    run_ubxtool(["-z", "CFG-UART2INPROT-RTCM3X,0"], port)
    
    # USB: entrada RTCM, salida solo UBX (sin NMEA para limpiar la vista)
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
    
    # Habilitar UBX-NAV-PVT a 5Hz por USB
    run_ubxtool(["-z", "CFG-MSGOUT-UBX_NAV_PVT_USB,5"], port)
    
    # RTCM 4072.0: Reference station PVT (requerido en cada época)
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE4072_0_UART2,1"], port)
    
    # RTCM MSM7: Observaciones de todas las constelaciones
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1077_UART2,1"], port)  # GPS MSM7
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1087_UART2,1"], port)  # GLONASS MSM7
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1097_UART2,1"], port)  # Galileo MSM7
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1127_UART2,1"], port)  # BeiDou MSM7
    
    # RTCM 1230: GLONASS code-phase biases (requerido si se usa GLONASS)
    run_ubxtool(["-z", "CFG-MSGOUT-RTCM_3X_TYPE1230_UART2,1"], port)
    
    # Guardar configuración
    run_ubxtool(["-p", "SAVE"], port)
    log("PR configurado")

def configure_mb(port: str):
    log(f"\nConfigurando MB en {port}")
    
    # Configurar tasa de navegación y medición a 5Hz (200ms) - debe coincidir con PR
    run_ubxtool(["-z", "CFG-RATE-MEAS,200"], port)  # medición cada 200ms = 5Hz
    run_ubxtool(["-z", "CFG-RATE-NAV,1"], port)     # 1 solución por medición
    
    # UART2: baudrate 460800, entrada RTCM
    run_ubxtool(["-z", "CFG-UART2-BAUDRATE,460800"], port)
    run_ubxtool(["-z", "CFG-UART2INPROT-RTCM3X,1"], port)
    run_ubxtool(["-z", "CFG-UART2OUTPROT-RTCM3X,0"], port)
    
    # UART1: deshabilitar NMEA (para limpiar salida)
    run_ubxtool(["-z", f"CFG-UART1-BAUDRATE,{MB_UART1_BAUD}"], port)
    run_ubxtool(["-z", "CFG-UART1OUTPROT-NMEA,0"], port)
    for msg in [
        "CFG-MSGOUT-NMEA_ID_VTG_UART1", "CFG-MSGOUT-NMEA_ID_GGA_UART1",
        "CFG-MSGOUT-NMEA_ID_RMC_UART1", "CFG-MSGOUT-NMEA_ID_GSA_UART1",
        "CFG-MSGOUT-NMEA_ID_GSV_UART1", "CFG-MSGOUT-NMEA_ID_GLL_UART1",
        "CFG-MSGOUT-NMEA_ID_GST_UART1", "CFG-MSGOUT-NMEA_ID_ZDA_UART1",
    ]:
        run_ubxtool(["-z", f"{msg},0"], port)

    # USB: salida solo UBX (sin NMEA)
    run_ubxtool(["-z", "CFG-USBOUTPROT-UBX,1"], port)
    run_ubxtool(["-z", "CFG-USBOUTPROT-NMEA,0"], port)
    for msg in [
        "CFG-MSGOUT-NMEA_ID_GGA_USB", "CFG-MSGOUT-NMEA_ID_RMC_USB",
        "CFG-MSGOUT-NMEA_ID_GSA_USB", "CFG-MSGOUT-NMEA_ID_VTG_USB",
        "CFG-MSGOUT-NMEA_ID_GSV_USB", "CFG-MSGOUT-NMEA_ID_GLL_USB",
        "CFG-MSGOUT-NMEA_ID_GST_USB", "CFG-MSGOUT-NMEA_ID_ZDA_USB",
    ]:
        run_ubxtool(["-z", f"{msg},0"], port)
    
    # Habilitar UBX-NAV-PVT a 5Hz por USB
    run_ubxtool(["-z", "CFG-MSGOUT-UBX_NAV_PVT_USB,5"], port)
    
    # Habilitar UBX-NAV-RELPOSNED a 5Hz por USB
    run_ubxtool(["-z", "CFG-MSGOUT-UBX_NAV_RELPOSNED_USB,5"], port)
    
    # PPS invertido (polarity=1) y habilitar TP1
    run_ubxtool(["-z", "CFG-TP-TP1_ENA,1"], port)
    run_ubxtool(["-z", "CFG-TP-POL_TP1,1"], port)
    # Refuerza con preset CFG-TP5: tpIdx=0, polarity=1
    run_ubxtool(["-p", "CFG-TP5,,,,,,,,,,,polarity=1"], port)
    
    # Guardar configuración
    run_ubxtool(["-p", "SAVE"], port)
    log("MB configurado")

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

def main():
    print_versions()
    log("=== CONFIGURACIÓN COMPLETA ZED-F9P ===")
    PR_PORT, MB_PORT = autodetect_ports()
    configure_pr(PR_PORT)
    configure_mb(MB_PORT)
    log("\nConfiguración finalizada correctamente.")

if __name__ == "__main__":
    main()

