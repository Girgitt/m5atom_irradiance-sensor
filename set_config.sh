#!/bin/bash

# this script accepts <config id> argument and does two things: 
# 1. backups config.h to config.h.bkp
# 2. replaces config.h with config_<config id>.h
# this allows to change sensor configuration to your local one before compilation
# to revert local config.h simply run restore_config.sh that will replace config.h with config.h.bkp

# Check if config_id was provided
if [ -z "$1" ]; then
  echo "Usage: $0 <config_id>"
  exit 1
fi

./restore_config.sh

CONFIG_ID="$1"
SRC_DIR="./src"
TARGET_FILE="$SRC_DIR/config.h"
BACKUP_FILE="$SRC_DIR/config.h.bkp"
SOURCE_FILE="$SRC_DIR/config_${CONFIG_ID}.h"

# Check if the new config file exists first
if [ ! -f "$SOURCE_FILE" ]; then
  echo "Error: $SOURCE_FILE does not exist. Aborting."
  exit 2
fi

# Create a backup of the current config.h only if the new one exists
if [ -f "$TARGET_FILE" ]; then
  cp "$TARGET_FILE" "$BACKUP_FILE"
  echo "Backup created: $BACKUP_FILE"
else
  echo "Warning: $TARGET_FILE does not exist. No backup made."
fi

# Replace config.h with the selected version
cp "$SOURCE_FILE" "$TARGET_FILE"
echo "Updated config.h with $SOURCE_FILE"
