#!/bin/bash
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

docker build -t srcp2 -f "${script_dir}/Dockerfile" "${script_dir}/../"
