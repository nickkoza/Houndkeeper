#!/bin/sh
# A release's own start-up check ([871]): the daemon stays up and answers its report within 30 s, as this release (a
# daemon that didn't restart still answers, as the old one), and is still up 10 s later (one that dies after its
# first answer fails).
ME=$(basename "$(cd "$(dirname "$0")" && pwd)")
ok() { systemctl is-active -q hk-hostd && python3 -I -c "import json,urllib.request,sys; r=json.load(urllib.request.urlopen('http://127.0.0.1:47100/health',timeout=2)); sys.exit(0 if r.get('release') == sys.argv[1] else 1)" "$ME" 2>/dev/null; }
for i in $(seq 1 30); do
  if ok && sleep 10 && ok; then
    exit 0; fi
  sleep 1
done
exit 1
