#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Configurador ZED-F9P (PR/MB) con autodetección (ACM0/ACM1) y recuperación de USB UBX.

- Autodetección: escanea /dev/ttyACM0 y /dev/ttyACM1, asigna PR al primero que responde MON-VER y MB al segundo.
- Si no responden, intenta habilitar UBX en USB (portID=3) vía CFG-PRT a ciegas y reintenta.
- Banner de versiones y niveles de log (--debug, --hex, --sniff).

Requisitos:
    pip install pyserial pyubx2
"""

from __future__ import annotations
import sys
import time
import argparse
import platform
import os
from dataclasses import dataclass
from serial import Serial, __version__ as pyserial_version
from pyubx2 import UBXMessage, UBXReader, SET, POLL, UBX_PROTOCOL, __version__ as pyubx2_version

# ------------------------------------------------------------
# PARÁMETROS
# ------------------------------------------------------------
CANDIDATE_PORTS = ("/dev/ttyACM0", "/dev/ttyACM1")
USB_BAUD = 9600               # Baud USB (CDC ACM lo ignora en teoría, pero el ZED-F9P puede tenerlo configurado)
MB_UART1_BAUD = 115200        # UART1 (MB) → salida NMEA
NMEA_RATE = 1                 # Hz

# IDs numéricos (compatibilidad)
UBX_CLASS_MON = 0x0A
UBX_ID_VER   = 0x04
UBX_CLASS_CFG = 0x06
UBX_ID_PRT   = 0x00
UBX_CLASS_ACK = 0x05
UBX_ID_ACK_ACK = 0x01
UBX_ID_ACK_NAK = 0x00

# ------------------------------------------------------------
# LOGGING / DEBUG
# ------------------------------------------------------------
@dataclass
class LogCfg:
    debug: bool = False
    hexout: bool = False
    sniff: bool = False

LOG = LogCfg()

def log(msg: str):
    print(msg)

def dbg(msg: str):
    if LOG.debug:
        print(f"[DEBUG] {msg}")

def hx(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)

def log_hex(prefix: str, data: bytes):
    if LOG.hexout:
        print(f"[HEX] {prefix}: {hx(data)}")


def decode_ack_raw(raw: bytes):
    """Devuelve tuple (ident, clsID, msgID) si el frame es ACK-ACK/NAK, else None."""
    if not raw or len(raw) < 10:
        return None
    if raw[0:2] != b"\xB5\x62":
        return None
    if raw[2] != 0x05:  # ACK class
        return None
    if raw[4] != 0x02:  # payload length 2
        return None
    ack_id = raw[3]
    cls_id = raw[6]
    msg_id = raw[7]
    ident = "ACK-ACK" if ack_id == 0x01 else "ACK-NAK" if ack_id == 0x00 else None
    if not ident:
        return None
    return ident, cls_id, msg_id

# ------------------------------------------------------------
# UTILIDADES UBX
# ------------------------------------------------------------
def open_gnss(port: str, baud: int = USB_BAUD) -> Serial:
    return Serial(port, baud, timeout=0.1)

def make_mon_ver_poll() -> UBXMessage:
    try:
        return UBXMessage("MON", "MON-VER", POLL)
    except Exception:
        return UBXMessage(UBX_CLASS_MON, UBX_ID_VER, POLL)

def read_ubx_until(ser: Serial, timeout_s: float = 2.5):
    rdr = UBXReader(ser, protfilter=UBX_PROTOCOL, quitonerror=0)
    end = time.time() + timeout_s
    last_read = time.time()
    
    while time.time() < end:
        # Si pasaron más de 0.5s sin leer nada, yield None y continuar
        if time.time() - last_read > 0.5:
            yield None, None
            last_read = time.time()
            
        try:
            raw, parsed = rdr.read()
            last_read = time.time()
            if raw:
                log_hex("RX", raw)
            yield raw, parsed
        except (EOFError, TimeoutError):
            # No hay datos disponibles
            yield None, None
            time.sleep(0.05)
            continue
        except Exception as e:
            dbg(f"Error leyendo UBX: {type(e).__name__}: {e}")
            yield None, None
            time.sleep(0.05)
            continue

def poll_mon_ver(ser: Serial, timeout_s: float = 1.0) -> str | None:
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass
    msg = make_mon_ver_poll()
    raw = msg.serialize()
    log_hex("TX MON-VER", raw)
    ser.write(raw); ser.flush()
    
    # Leer directamente del puerto sin UBXReader para evitar bloqueos
    end = time.time() + timeout_s
    buffer = b""
    
    while time.time() < end:
        try:
            chunk = ser.read(128)  # leer lo que haya disponible
            if not chunk:
                time.sleep(0.05)
                continue
            buffer += chunk
            log_hex("RX", chunk)
            
            # Buscar respuesta MON-VER en el buffer
            if b"\xb5\x62" in buffer:  # UBX header
                try:
                    # Intentar parsear con UBXReader desde el buffer
                    from io import BytesIO
                    bio = BytesIO(buffer)
                    rdr = UBXReader(bio, protfilter=UBX_PROTOCOL, quitonerror=0)
                    _, parsed = rdr.read()
                    if parsed and getattr(parsed, "identity", "") == "MON-VER":
                        sw = getattr(parsed, "swVersion", None)
                        return sw if sw else "MON-VER OK"
                except Exception as e:
                    dbg(f"Parse intento: {e}")
                    continue
        except Exception as e:
            dbg(f"Error leyendo: {e}")
            time.sleep(0.05)
            continue
    
    return None

def await_ack(ser: Serial, expect_class: int | None = None, expect_id: int | None = None, timeout_s: float = 2.5) -> bool | None:
    for raw, parsed in read_ubx_until(ser, timeout_s=timeout_s):
        if parsed:
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
        ack = decode_ack_raw(raw)
        if ack:
            ident, cls_id, msg_id = ack
            cls_match = expect_class is None or cls_id == expect_class
            id_match  = expect_id   is None or msg_id == expect_id
            if not (cls_match and id_match):
                continue
            if ident == "ACK-ACK":
                dbg(f"ACK-ACK (raw fallback) cls=0x{cls_id:02X}, id=0x{msg_id:02X}")
                return True
            else:
                dbg(f"ACK-NAK (raw fallback) cls=0x{cls_id:02X}, id=0x{msg_id:02X}")
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
        ok, _ = check_connection(p, retries=2, sniff_before=True)
        if ok:
            responding.append(p)

    if len(responding) < 2:
        log("\nError: no se detectaron dos ZED-F9P respondiendo en /dev/ttyACM0 y /dev/ttyACM1.")
        log("Tips: ¿gpsd activo?  sudo systemctl stop gpsd.socket gpsd")
        log("      ¿ModemManager activo? sudo systemctl stop ModemManager")
        log("      ¿USB UBX deshabilitado por config previa? Este script intenta re-habilitarlo (CFG-PRT USB).")
        sys.exit(1)

    pr_port, mb_port = responding[0], responding[1]
    log(f"\nAsignación automática: PR={pr_port}  MB={mb_port}")
    return pr_port, mb_port

# ------------------------------------------------------------
# CONFIGURACIONES
# ------------------------------------------------------------
def set_pps_inverted(port: str):
    log(f"\nConfigurando PPS invertido (CFG-TP5) en {port}")
    ser = open_gnss(port)
    msg = UBXMessage(
        "CFG", "CFG-TP5",
        SET,
        tpIdx=0,
        version=0,
        antCableDelay=0,
        rfGroupDelay=0,
        freqPeriod=0,
        freqPeriodLock=0,
        pulseLen=1_000_000,
        pulseLenLock=1_000_000,
        polarity=1,  # flanco de bajada
        gridUtc=0,
        syncMode=0,
        timeGrid=0,
        sigEdge=0,
        flags=0
    )
    send_and_wait_ack(ser, msg)
    ser.close()

def configure_pr(port: str):
    log(f"\nConfigurando POSITION RECEIVER (PR) en {port}")
    ser = open_gnss(port)
    # UART2 → salida RTCM3
    msg_prt = UBXMessage(
        "CFG", "CFG-PRT",
        SET,
        portID=1,           # UART2
        mode=0x08D0,        # 8N1
        baudRate=460800,
        inProtoMask=0,
        outProtoMask=4      # RTCM3
    )
    send_and_wait_ack(ser, msg_prt)
    # Habilitar un RTCM genérico (ejemplo MSM)
    msg_rtcm = UBXMessage("CFG", "CFG-MSG", SET, msgClass=0xF5, msgID=0x05, rate=1)
    send_and_wait_ack(ser, msg_rtcm)
    ser.close()
    log("PR configurado")

def configure_mb(port: str):
    log(f"\nConfigurando MOVING BASELINE RECEIVER (MB) en {port}")
    ser = open_gnss(port)
    # UART1: salida NMEA
    msg_prt1 = UBXMessage(
        "CFG", "CFG-PRT",
        SET,
        portID=0,              # UART1
        mode=0x08D0,           # 8N1
        baudRate=MB_UART1_BAUD,
        inProtoMask=0,
        outProtoMask=2         # NMEA
    )
    send_and_wait_ack(ser, msg_prt1)
    # UART2: entrada RTCM3
    msg_prt2 = UBXMessage(
        "CFG", "CFG-PRT",
        SET,
        portID=1,              # UART2
        mode=0x08D0,
        baudRate=460800,
        inProtoMask=4,         # RTCM3
        outProtoMask=0
    )
    send_and_wait_ack(ser, msg_prt2)
    # NMEA: habilitar GGA, RMC, GSA
    def enable_nmea(msgid: int):
        msg = UBXMessage("CFG", "CFG-MSG", SET, msgClass=0xF0, msgID=msgid, rate=NMEA_RATE)
        send_and_wait_ack(ser, msg)
    enable_nmea(0x00)  # GGA
    enable_nmea(0x04)  # RMC
    enable_nmea(0x02)  # GSA
    ser.close()
    log("MB configurado")


def poll_cfg_prt(port: str, port_id: int):
    """Lee CFG-PRT de un portID y devuelve el mensaje UBX parseado o None."""
    ser = open_gnss(port)
    msg = UBXMessage("CFG", "CFG-PRT", POLL, portID=port_id)
    raw = msg.serialize()
    log_hex(f"TX CFG-PRT({port_id})", raw)
    ser.write(raw); ser.flush()
    for _, parsed in read_ubx_until(ser, timeout_s=1.0):
        if parsed and getattr(parsed, "identity", "") == "CFG-PRT":
            ser.close()
            return parsed
    ser.close()
    return None


def poll_cfg_tp5(port: str, tp_idx: int = 0):
    """Lee CFG-TP5 (timing pulse) y devuelve el mensaje UBX parseado o None."""
    ser = open_gnss(port)
    msg = UBXMessage("CFG", "CFG-TP5", POLL, tpIdx=tp_idx)
    raw = msg.serialize()
    log_hex(f"TX CFG-TP5({tp_idx})", raw)
    ser.write(raw); ser.flush()
    for _, parsed in read_ubx_until(ser, timeout_s=1.0):
        if parsed and getattr(parsed, "identity", "") == "CFG-TP5":
            ser.close()
            return parsed
    ser.close()
    return None

def save_config(port: str):
    log(f"Guardando configuración en {port}")
    ser = open_gnss(port)
    # clearMask, saveMask, loadMask deben ser bytes (4 bytes cada uno)
    msg = UBXMessage(
        "CFG", "CFG-CFG", SET,
        clearMask=b'\x00\x00\x00\x00',
        saveMask=b'\xff\xff\x00\x00',
        loadMask=b'\x00\x00\x00\x00'
    )
    send_and_wait_ack(ser, msg)
    ser.close()


def summarize_prt(msg):
    try:
        baud = getattr(msg, "baudRate", None)
        inmask = getattr(msg, "inProtoMask", None)
        outmask = getattr(msg, "outProtoMask", None)
        return f"baud={baud} inMask=0x{inmask:02X} outMask=0x{outmask:02X}"
    except Exception:
        return "(no se pudo formatear CFG-PRT)"


def summarize_tp5(msg):
    try:
        pol = getattr(msg, "polarity", None)
        pulse = getattr(msg, "pulseLen", None)
        return f"polarity={pol} pulseLen={pulse}"
    except Exception:
        return "(no se pudo formatear CFG-TP5)"


def verify_configurations(pr_port: str, mb_port: str):
    log("\nVerificando configuración aplicada…")

    pr_tp5 = poll_cfg_tp5(pr_port, tp_idx=0)
    log(f"PR CFG-TP5: {summarize_tp5(pr_tp5)}" if pr_tp5 else "PR CFG-TP5: sin respuesta")
    pr_prt = poll_cfg_prt(pr_port, port_id=1)
    log(f"PR CFG-PRT UART2: {summarize_prt(pr_prt)}" if pr_prt else "PR CFG-PRT UART2: sin respuesta")

    mb_tp5 = poll_cfg_tp5(mb_port, tp_idx=0)
    log(f"MB CFG-TP5: {summarize_tp5(mb_tp5)}" if mb_tp5 else "MB CFG-TP5: sin respuesta")
    mb_prt1 = poll_cfg_prt(mb_port, port_id=0)
    log(f"MB CFG-PRT UART1: {summarize_prt(mb_prt1)}" if mb_prt1 else "MB CFG-PRT UART1: sin respuesta")
    mb_prt2 = poll_cfg_prt(mb_port, port_id=1)
    log(f"MB CFG-PRT UART2: {summarize_prt(mb_prt2)}" if mb_prt2 else "MB CFG-PRT UART2: sin respuesta")

# ------------------------------------------------------------
# BANNER VERSIONES
# ------------------------------------------------------------
def print_versions():
    log("=== ENTORNO / VERSIONES ===")
    log(f"Python:         {platform.python_version()} ({sys.executable})")
    log(f"SO:             {platform.system()} {platform.release()}  ({platform.platform()})")
    log(f"pyserial:       {pyserial_version}")
    log(f"pyubx2:         {pyubx2_version}")
    log(f"USER:           {os.environ.get('USER') or os.environ.get('USERNAME')}")
    log(f"PWD:            {os.getcwd()}")
    log("============================\n")

# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Config ZED-F9P PR/MB con autodetección (ACM0/ACM1)")
    ap.add_argument("--debug", action="store_true", help="Mensajes de debug")
    ap.add_argument("--hex", action="store_true", help="Dump HEX de frames UBX TX/RX")
    ap.add_argument("--sniff", action="store_true", help="Muestra 1.5s de tráfico bruto por puerto antes de probar")
    args = ap.parse_args()
    LOG.debug = args.debug
    LOG.hexout = args.hex
    LOG.sniff = args.sniff

    print_versions()

    log("=== CONFIGURACIÓN COMPLETA ZED-F9P ===")
    PR_PORT, MB_PORT = autodetect_ports()

    set_pps_inverted(PR_PORT)
    set_pps_inverted(MB_PORT)

    configure_pr(PR_PORT)
    configure_mb(MB_PORT)

    save_config(PR_PORT)
    save_config(MB_PORT)

    verify_configurations(PR_PORT, MB_PORT)

    log("\nConfiguración finalizada correctamente.")

if __name__ == "__main__":
    main()

