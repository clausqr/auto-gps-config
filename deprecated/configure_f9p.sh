#!/bin/bash
# Configurador ZED-F9P (PR/MB) con autodetección usando ubxtool

set -e

# Colores
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[INFO]${NC} $*"
}

error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

warn() {
    echo -e "${YELLOW}[WARN]${NC} $*"
}

# Verificar que ubxtool está instalado
if ! command -v ubxtool &> /dev/null; then
    error "ubxtool no está instalado. Instalar con: pip install gpsd-py3"
    exit 1
fi

# Puertos candidatos
PORTS=("/dev/ttyACM0" "/dev/ttyACM1")

# Función para chequear si un puerto responde a MON-VER
check_port() {
    local port=$1
    log "Chequeando $port..."
    
    # Intentar leer versión con timeout corto
    if timeout 2s ubxtool -f "$port" -p MON-VER 2>/dev/null | grep -q "UBX-MON-VER"; then
        return 0
    fi
    return 1
}

# Autodetección de puertos
log "=== AUTODETECCIÓN DE PUERTOS ==="
RESPONDING=()

for port in "${PORTS[@]}"; do
    if [ -e "$port" ]; then
        if check_port "$port"; then
            log "✓ $port responde"
            RESPONDING+=("$port")
        else
            warn "✗ $port no responde"
        fi
    else
        warn "✗ $port no existe"
    fi
done

# Verificar que tengamos 2 puertos
if [ ${#RESPONDING[@]} -lt 2 ]; then
    error "Se necesitan 2 ZED-F9P pero solo se detectaron ${#RESPONDING[@]}"
    error ""
    error "Tips:"
    error "  - ¿gpsd activo? sudo systemctl stop gpsd.socket gpsd"
    error "  - ¿ModemManager activo? sudo systemctl stop ModemManager"
    error "  - ¿USB conectado correctamente?"
    exit 1
fi

PR_PORT="${RESPONDING[0]}"
MB_PORT="${RESPONDING[1]}"

log ""
log "Asignación automática:"
log "  PR (Position Receiver) = $PR_PORT"
log "  MB (Moving Baseline)   = $MB_PORT"
log ""

# Función para configurar PPS invertido
configure_pps_inverted() {
    local port=$1
    local name=$2
    log "Configurando PPS invertido en $name ($port)..."
    
    # CFG-TP5: Time Pulse 0
    # Formato: port, message, fields
    # Para ahora solo reportamos que lo intentamos
    warn "PPS invertido requiere CFG-TP5 (manual o implementar en pyubx2)"
}

# Función para configurar PR (Position Receiver)
configure_pr() {
    local port=$1
    log "=== CONFIGURANDO PR ($port) ==="
    
    # Solo hacer polling de MON-VER para verificar
    ubxtool -f "$port" -p MON-VER
    
    log "✓ PR detectado (configuración manual recomendada)"
}

# Función para configurar MB (Moving Baseline)
configure_mb() {
    local port=$1
    log "=== CONFIGURANDO MB ($port) ==="
    
    # Solo hacer polling de MON-VER para verificar
    ubxtool -f "$port" -p MON-VER
    
    log "✓ MB detectado (configuración manual recomendada)"
}

# Función para guardar configuración
save_config() {
    local port=$1
    local name=$2
    log "Verificando configuración en $name ($port)..."
    ubxtool -f "$port" -p MON-VER
}

# Ejecutar configuraciones
log ""
log "=== INICIANDO CONFIGURACIÓN ==="
log ""

# Configurar PPS invertido en ambos
configure_pps_inverted "$PR_PORT" "PR"
configure_pps_inverted "$MB_PORT" "MB"

# Configurar cada receptor
configure_pr "$PR_PORT"
log ""
configure_mb "$MB_PORT"

# Guardar configuraciones
log ""
log "=== GUARDANDO CONFIGURACIONES ==="
save_config "$PR_PORT" "PR"
save_config "$MB_PORT" "MB"

log ""
log "=== ✓ AUTODETECCIÓN COMPLETADA ==="
log ""
log "Resumen:"
log "  PR: $PR_PORT"
log "  MB: $MB_PORT"
log ""
log "Para configuración completa, usar ubxtool manualmente:"
log "  ubxtool -f $PR_PORT -z CFG-PRT,..."
log "  ubxtool -f $MB_PORT -z CFG-PRT,..."
log ""
