#!/bin/bash

FILE_PATH=".serial_port"


# 3. Execute your command
# $(<file) reads the file content as a single string (command substitution)
# "$@" passes all remaining arguments exactly as they were provided
port=$(xargs < "$FILE_PATH")
idf.py flash -p "$port" "$@"

