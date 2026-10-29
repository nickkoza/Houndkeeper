# The robot network: a network with no way out

For the dog-first-boot spec ([MTv]): [hNo] a segment with no route to the internet, [VVJ] isolation shown, [x20] one
clock. Design: decision [ZaY], a separate network on the house router, closed by its firewall.

The house's specifics (the network names, addresses, and the router's own configuration file) live in `real/.env`
and `real/local/`, both gitignored; `real/.env.example` lists them. Below, `$ROBOT_SSID`, `$ROBOT_NET`, `$ROBOT_IP`,
`$ARM_HOST_IP`, `$LAPTOP_IP` and `$ROUTER_ON_ROBOT_NET` are those values.

## 1. What the router must be set to

- A second wireless network, `$ROBOT_SSID`, 2.4 GHz, on its own bridge and subnet `$ROBOT_NET`, with DHCP: fixed
  addresses for the robot and the arm host; the DNS server it hands out is the laptop; the time server is the router.
- Firewall, above every other rule:
  1. the operators (the laptop; a router address list its fixed DHCP leases fill, so no house address is written into a rule:
     house addresses aren't stable) may open connections into `$ROBOT_NET`;
  2. `$ROBOT_NET` may reach the operators (WebRTC both ways, the arm host's reports and microphone);
  3. anything else from `$ROBOT_NET` is logged, then dropped;
  4. nothing else may reach `$ROBOT_NET`;
  5. the router answers `$ROBOT_NET` only for addresses (DHCP) and time (NTP); anything else to it is logged, dropped;
  6. no IPv6 in or out of `$ROBOT_NET`.
- The laptop has fixed DHCP leases on the house networks (`$LAPTOP_IP`; by its MAC, `$LAPTOP_MAC`), each putting it in the
  operators list; the robot's DNS points at those addresses (the logging resolver, below).

The router's own configuration for all of this is the file `$ROUTER_SCRIPT` (local). Back up the router's current
config before applying it; then set the robot network's passphrase to `$ROBOT_PSK` (from `real/.env`, generated; it
is never in a committed file, [Ad4]).

## 2. The laptop's two logs: names looked up, and the router's firewall log

`real/bringup/robot_logs.py` keeps both, each as a systemd service; the laptop's addresses and the log folder come
from `real/.env` (`LAPTOP_HOUSE_IPS`, `ROBOT_LOG_DIR`, default `real/local/logs/`, and `ROBOT_SYSLOG_PORT`), so
nothing house-specific is in the script or the units.

- **`hk-robot-dns`**: dnsmasq answering every name "no such domain" (`--address=/#/`, no upstream, no cache) and
  logging each query to `robot-dns.log`. It listens only on the laptop's house addresses (`--bind-dynamic`, so the
  address of whichever house network the laptop is on is taken up when it appears), never on 0.0.0.0 or the
  loopback: systemd-resolved keeps 127.0.0.53/54 and the laptop's own DNS is untouched. Needs `dnsmasq` (the
  `dnsmasq-base` package is enough: the binary, no system service of its own to collide on port 53). Runs as the
  laptop's user with only the capability to bind port 53.
- **`hk-robot-syslog`**: receives the router's firewall log of the robot network ("robot-out " / "robot-in ")
  by remote syslog on UDP `$ROBOT_SYSLOG_PORT` (5514: no privilege), on the same addresses, and appends each line,
  stamped with the laptop's time and the sender, to `router.log`. The router's side is the local file
  `real/local/router-logging.rsc`: one remote action per laptop address, plus a bounded file on the router's
  flash as a backstop for when the laptop sleeps.

Install (once; the units are templates, filled with this checkout's path and the user's name):

    for u in hk-robot-dns hk-robot-syslog; do
      sed -e "s|@REPO@|$PWD|g" -e "s|@USER@|$USER|g" real/bringup/$u.service | sudo tee /etc/systemd/system/$u.service >/dev/null
    done
    sudo systemctl daemon-reload
    sudo ufw allow in proto udp from $ROBOT_NET to $LAPTOP_IP port 53 comment 'robot network DNS log'    # each house address
    sudo ufw allow in proto tcp from $ROBOT_NET to $LAPTOP_IP port 53 comment 'robot network DNS log'
    sudo ufw allow in proto udp from <the router's house-side address> to $LAPTOP_IP port $ROBOT_SYSLOG_PORT comment 'router log'

Run them while the robot is on the segment (`sudo systemctl enable --now hk-robot-dns hk-robot-syslog` to keep them
across reboots; `stop` and `disable` when done). Read: `tail -f real/local/logs/robot-dns.log real/local/logs/router.log`.
`python3 real/bringup/robot_logs.py show` prints what the services will run. Checked by `tests/test_robot_logs.py`.

## 3. Prove it ([VVJ]): after every change to the router

From the arm host (or any device joined to `$ROBOT_SSID`):

    getent hosts example.com          # must fail (and appear in the laptop's DNS log)
    curl -m 5 https://1.1.1.1          # must fail (and appear in the router's log)
    ping -c 2 $LAPTOP_IP               # must work
    ping -c 2 <the router's house-side address>   # must fail

From another house device not on `$ROBOT_SSID`: `ping $ROBOT_IP` must fail; from the laptop it must work. Then export
the router's config for the robot network and diff it against `$ROUTER_SCRIPT`.

### The router's side, checked every few minutes

`real/bringup/isolation_check.py` reads the router over SSH, read-only (prints only), and checks the design above
still holds: the Robot interface list holds only the robot network's bridge, and neither it nor Guest is in the LAN
list; the robot network's rules exist, are enabled, exactly as written, and are the first rules of their chains in
the designed order (IPv6 drops included); no other rule carries the robot network's tag (an open online window shows
here); the operators list holds single hosts; telnet, ftp, www, www-ssl, api, api-ssl and reverse-proxy are off, ssh
and winbox open only to the private network. It also reads the drop rules' counters (attempts dropped since the
router booted). The result goes to `real/local/isolation.json`, whose one-line summary ("passed: ..." or
"FAILED: <what>") the phone cockpit's Status page shows. It trusts only the router's SSH host key recorded in
`$ROUTER_KNOWN_HOSTS` (default `real/local/router_known_hosts`): a missing or different key refuses before the
password is sent. Settings from `real/.env` (`.env.example` lists them); it needs a Python with paramiko.
Checked by `tests/test_isolation_check.py`, on the router's own output saved with placeholders.

Every 5 minutes, as a systemd user timer (no sudo; the units are templates):

    for u in service timer; do
      sed -e "s|@REPO@|$PWD|g" -e "s|@PYTHON@|<a python with paramiko>|g" \
        real/bringup/hk-isolation-check.$u > ~/.config/systemd/user/hk-isolation-check.$u
    done
    systemctl --user daemon-reload && systemctl --user enable --now hk-isolation-check.timer
    loginctl enable-linger $USER     # optional: keep running while logged out

Or a cron line instead: `*/5 * * * * cd <repo> && <python> real/bringup/isolation_check.py >/dev/null 2>&1`.

Proven 3 Oct 2026 from a laptop joined to the robot network: an address by DHCP, time from the router; no name resolved,
no internet, neither side of the router, no Guest or Private device; each attempt in the router's log.

The router keeps its log in memory (a ring of ~1,000 lines): before the robot's first day, send it to the laptop
(section 2), or the early attempts scroll away. Leave the robot on the segment for a day; read the router's log of dropped attempts and the laptop's DNS log. Every
name and address it tried, and how often, goes into the first-boot record (kept locally, like the rest).

## Notes

- The robot joins `$ROBOT_SSID` during the online window (first-boot step 1), set in the Unitree app.
- The router keeps its own internet time and serves it to the segment; the laptop follows the same internet time,
  so they share one clock ([x20]).
- A phone with the Unitree app can join `$ROBOT_SSID` to reach the robot locally; whether the app works with no
  internet there is an open question in [MTv].
