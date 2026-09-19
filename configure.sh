#!/usr/bin/env bash
set -euo pipefail

# Configure from the project directory, regardless of the caller's location.
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cmake -S "$project_dir" -B "$project_dir/build" "$@"
