#!/bin/bash
if [ -z "$IDF_PATH" ]; then
    echo "Error: ESP-IDF environment not loaded. Run '. ./activate_scripts.sh' first." >&2
    exit 1
fi
idf.py build "$@"

