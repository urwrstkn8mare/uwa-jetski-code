#!/bin/bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "ERROR: Script must be sourced, not ran directly"
    exit 1
else
	SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
	export PATH="$SCRIPT_DIR/scripts:$PATH"
	alias build='build.sh'
	alias choose_port='choose_port.py'
	alias clean='clean.py'
	alias monitor='monitor.py'
	alias flash='flash.py'
	echo "Source successful"
fi

