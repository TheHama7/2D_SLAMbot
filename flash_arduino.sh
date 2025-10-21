#!/bin/bash
set -e

ARDUINO_CLI=${ARDUINO_CLI:-arduino-cli}
PORT=${1:-/dev/ttyACM0}
BOARD=${2:-arduino:avr:uno}
SKETCH=Thesis/ArduinoMC/ArduinoMC.ino

echo "🧩 Compiling Arduino sketch..."
$ARDUINO_CLI compile --fqbn $BOARD $SKETCH

echo "⚡ Uploading to $PORT..."
$ARDUINO_CLI upload -p $PORT --fqbn $BOARD $SKETCH

echo "✅ Upload complete!"
