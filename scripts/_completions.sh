#!/usr/bin/env bash
# shellcheck shell=bash
#
# Tab completion for project utility scripts.
# Supports bash and zsh when sourced from activate_scripts.sh.

_uwa_list_projects() {
    local repo_root projects_dir script_dir source_path
    repo_root="${UWA_JETSKI_REPO_ROOT:-}"
    if [[ -z "${repo_root}" ]]; then
        if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
            source_path="${BASH_SOURCE[0]}"
        else
            source_path="$0"
        fi
        script_dir="$(cd "$(dirname "${source_path}")" && pwd)"
        repo_root="$(cd "${script_dir}/.." && pwd)"
    fi
    projects_dir="${repo_root}/projects"

    if [[ -d "${projects_dir}" ]]; then
        for d in "${projects_dir}"/*; do
            [[ -d "${d}" && -f "${d}/CMakeLists.txt" ]] && basename "${d}"
        done
    fi
}

_uwa_complete_project_value() {
    local current="$1"
    local base="${current##*,}"
    local prefix=""
    if [[ "${current}" == *,* ]]; then
        prefix="${current%,*},"
    fi

    local projects
    projects="$(_uwa_list_projects)"
    local matches
    matches=$(compgen -W "${projects}" -- "${base}")

    while IFS= read -r m; do
        [[ -n "${m}" ]] && printf '%s\n' "${prefix}${m}"
    done <<< "${matches}"
}

_uwa_script_completion_bash() {
    local cur prev
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    case "${prev}" in
        -p|--project)
            COMPREPLY=()
            while IFS= read -r item; do
                COMPREPLY+=("${item}")
            done < <(_uwa_complete_project_value "${cur}")
            return 0
            ;;
    esac

    COMPREPLY=($(compgen -W "-h --help -v --verbose -p --project" -- "${cur}"))
    return 0
}

_uwa_script_completion_zsh() {
    local curcontext="$curcontext" state
    typeset -A opt_args
    _arguments -C \
        '(-h --help)'{-h,--help}'[show help]' \
        '(-v --verbose)'{-v,--verbose}'[enable verbose logging]' \
        '(-p --project)'{-p,--project}'[target project name(s), comma-separated]:project:_uwa_projects_zsh' \
        '*:arg:->args'
}

_uwa_projects_zsh() {
    local projects
    projects=("${(@f)$(_uwa_list_projects)}")

    local current_word base prefix
    current_word="${words[CURRENT]}"
    base="${current_word##*,}"
    prefix=""
    if [[ "${current_word}" == *,* ]]; then
        prefix="${current_word%,*},"
    fi

    local -a out
    out=()
    local p
    for p in "${projects[@]}"; do
        [[ "${p}" == "${base}"* ]] && out+=("${prefix}${p}")
    done
    compadd -a out
}

_uwa_register_completions() {
    local commands=(
        build build.py
        save_defconfig save_defconfig.py
        clean clean.py
        flash flash.py
        monitor monitor.py
        choose_port choose_port.py
        setup_clangd setup_clangd.py
    )

    if [[ -n "${BASH_VERSION:-}" ]]; then
        local cmd
        for cmd in "${commands[@]}"; do
            complete -F _uwa_script_completion_bash "${cmd}"
        done
    elif [[ -n "${ZSH_VERSION:-}" ]]; then
        autoload -Uz compinit
        compinit -i
        setopt complete_aliases
        local cmd
        for cmd in "${commands[@]}"; do
            compdef _uwa_script_completion_zsh "${cmd}"
        done
    fi
}
