#!/usr/bin/env bash
# Commit and push everything in this workspace to GitHub.
#
#   ./gitsync.sh "added joystick teleop launch file"
#   ./gitsync.sh                      # uses a timestamped message
#
set -euo pipefail
cd "$(dirname "$0")"

branch="$(git symbolic-ref --short HEAD)"
msg="${1:-Update workspace ($(date '+%Y-%m-%d %H:%M'))}"

git add -A

if git diff --cached --quiet; then
  echo "Nothing new to commit."
else
  git commit -m "$msg"
fi

git push -u origin "$branch"
