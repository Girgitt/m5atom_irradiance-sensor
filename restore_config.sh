#!/bin/bash

SRC_DIR="./src"
BACKUP_FILE="$SRC_DIR/config.h.bkp"
TARGET_FILE="$SRC_DIR/config.h"

# Check if backup exists
if [ -f "$BACKUP_FILE" ]; then
  cp "$BACKUP_FILE" "$TARGET_FILE"
  echo "Restored $BACKUP_FILE to $TARGET_FILE"
else
  echo "Error: Backup file $BACKUP_FILE does not exist."
  exit 1
fi
