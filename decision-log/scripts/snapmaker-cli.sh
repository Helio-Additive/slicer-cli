#!/bin/bash
# snapmaker-cli.sh <args...> — run Snapmaker Orca v2.4.1's own command line headless, with all network access denied
# (it starts a Sentry session on every run; D-006), a 15-minute limit, and its own data dir so the user's settings are untouched.
W="$(cd "$(dirname "$0")/../.." && pwd)"
SM="$W/work/snapmaker-app/Snapmaker Orca.app/Contents/MacOS/Snapmaker_Orca"
mkdir -p "$W/work/smapp-datadir"
exec "$W/decision-log/scripts/with-timeout.sh" 900 /usr/bin/sandbox-exec -p '(version 1)(allow default)(deny network*)' "$SM" --datadir "$W/work/smapp-datadir" "$@"
