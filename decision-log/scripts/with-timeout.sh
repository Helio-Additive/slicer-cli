#!/bin/bash
# with-timeout.sh <seconds> <cmd...>  — macOS has no `timeout`; kill the command after N seconds.
s=$1; shift; "$@" & pid=$!
( sleep "$s"; kill -TERM $pid 2>/dev/null; sleep 5; kill -KILL $pid 2>/dev/null ) & w=$!
wait $pid; rc=$?; kill $w 2>/dev/null; exit $rc
