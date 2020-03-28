#!/bin/bash
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

docker build -t src -f "${script_dir}/Dockerfile" "${script_dir}/../"
