#!/bin/bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo "ERROR: Script must be sourced, not run directly" >&2
    exit 1
fi

if ! command -v idf.py >/dev/null 2>&1; then
    echo "ERROR: activate ESP-IDF first (idf.py not found in PATH)" >&2
    return 1
fi

SCRIPT_DIR="$(git rev-parse --show-toplevel)/scripts"
export UWA_JETSKI_REPO_ROOT="$(git rev-parse --show-toplevel)"
export PATH="$SCRIPT_DIR:$PATH"
alias build='build.py'
alias choose_port='choose_port.py'
alias clean='clean.py'
alias monitor='monitor.py'
alias flash='flash.py'
alias save_defconfig='save_defconfig.py'

if [[ -f "$SCRIPT_DIR/_completions.sh" ]]; then
    # shellcheck source=/dev/null
    source "$SCRIPT_DIR/_completions.sh"
    _uwa_register_completions
fi

echo "$SCRIPT_DIR added to \$PATH"
echo "Extension-less aliases created (build, choose_port, clean, monitor, flash, save_defconfig)"
