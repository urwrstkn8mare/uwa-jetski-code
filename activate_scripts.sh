#!/bin/bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
	echo "ERROR: Script must be sourced, not ran directly"
	exit 1
fi

if ! idf.py --version; then
	echo "ERROR: Script must be sourced within an ESP-IDF environment! (otherwise what's the point)"
else
	SCRIPT_DIR="$(git rev-parse --show-toplevel)/scripts"
	export PATH="$SCRIPT_DIR:$PATH"
	echo "$SCRIPT_DIR added to \$PATH"
	alias build='build.sh'
	alias choose_port='choose_port.py'
	alias clean='clean.py'
	alias monitor='monitor.py'
	alias flash='flash.py'
	echo "Extension-less aliases created (build, choose_port, clean, monitor, flash)"
fi
