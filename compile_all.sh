#!/bin/bash

cd "$(dirname "$0")" || exit 1

# Check for PlatformIO
if ! command -v pio &>/dev/null; then
  echo "Error: PlatformIO CLI not found."
  exit 1
fi

echo "Compiling all environments defined in platformio.ini..."

# Extract env names only (filter out warnings or noise)
envs=$(grep -oP '^\[env:\K[^\]]+' platformio.ini)

if [ -z "$envs" ]; then
  echo "No valid environments found."
  exit 1
fi

# Compile each environment
for env in $envs; do
  echo "==> Building environment: $env"
  if ! pio run -e "$env"; then
    echo "❌ Build failed for environment: $env"
    exit 1
  fi
done

echo "✅ All environments built successfully."
