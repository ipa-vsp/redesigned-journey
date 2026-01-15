#!/bin/sh
set -e

if [ -n "$VIRTUAL_ENV" ] && [ -f "$VIRTUAL_ENV/bin/activate" ]; then
  # Ensure the venv is active for interactive shells and commands.
  . "$VIRTUAL_ENV/bin/activate"
fi

exec "$@"
