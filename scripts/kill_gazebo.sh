#!/usr/bin/env bash
# Mata todos los procesos Gazebo y limpia memoria compartida.
# Usar cuando gzserver queda zombi tras Ctrl+C y el siguiente
# lanzamiento falla con exit code 255.

echo "Matando gzserver y gzclient..."
pkill -9 -f gzserver 2>/dev/null && echo "  gzserver killed" || echo "  gzserver not running"
pkill -9 -f gzclient 2>/dev/null && echo "  gzclient killed" || echo "  gzclient not running"
pkill -9 -f gazebo    2>/dev/null && echo "  gazebo killed"   || true

# Segmentos de memoria compartida de Gazebo
if ls /dev/shm/gazebo* 2>/dev/null; then
    rm -f /dev/shm/gazebo*
    echo "  Shared memory cleared"
fi

echo "Listo. Espera 2 segundos antes de relanzar."
sleep 2
