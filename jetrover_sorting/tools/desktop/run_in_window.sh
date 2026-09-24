#!/usr/bin/env bash
# Used by the desktop icons: runs a script and keeps the window open briefly
# afterwards so its last messages can be read without a keyboard.
"$@"
echo
echo "(this window closes in 15 seconds)"
sleep 15
