# Day one: everything in order

The day Barrymore arrives, start to finish, as commands. Each step names the page with the detail and the reasons;
house specifics (addresses, names, keys) are in `real/.env` and `real/local/`, never here. Run from the repository
root; `$PY` is the DimOS venv's python (`../dimos/.venv/bin/python`, the DimOS checkout beside this repo), `$RPY` the router tools' venv
(`real/local/venv/bin/python`, with paramiko).

## The day before

1. **The robot network is closed.** `$RPY real/bringup/isolation_check.py` must print `passed`; its timer
   (`real/bringup/hk-isolation-check.timer`, robot-network.md §3) keeps `real/local/isolation.json` fresh, which the
   phone's Status page and every real recording session read ([VVJ]).
2. **The laptop logs what the robot tries.** The DNS logger and the router-log receiver are running
   (`systemctl status hk-robot-dns hk-robot-syslog`; robot-network.md §2), and the router sends its robot log to
   the laptop (`real/local/router-logging.rsc`).
3. **The phone reaches the cockpit.** The relay runs on the laptop's Tailscale address with its certificate and a
   viewer token (phone-cockpit [rhP], below), and the phone has it bookmarked. Try it against the sim first:
   `dimos --simulation mujoco run --local-relay unitree-go2 houndkeeper.phone-cockpit --extensions-dir cockpit/extensions`.
4. **Room to record.** `hk-data ls --real` is empty or synced; `df -h` shows well over 50 GB free where DimOS
   records (sessions refuse below it, [6xc]); `hk-data prune` lists what could go if not.
5. **The stops are known** (first-boot.md §3, [U4p]): the cockpit's STOP and its dead-man (200 ms, tested
   `real/tests/deadman_sim.sh`), a gamepad held to drive if one is at hand (hold LB, B stops), and the robot's power
   button, the one that needs no software.

## The morning

6. **Unbox and charge**, supervised (first-boot.md §0).
7. **Bind, only if it isn't** (first-boot.md §1, [zAo], [wZV]): the contained online window
   (`real/local/online-window.rsc`): names learned with the window closed, Unitree's only, HTTPS, capped, closing
   itself after 30 minutes. Record the firmware version. Then the LAN key on the laptop:
   `unitree-fetch-aes-key --email <account> --sn <serial>` into DimOS's local secret config (never the repo, [Ad4]).
8. **On the robot network**: fixed leases for the robot and the arm host by their MACs (the temporary router user is
   kept for this, then deleted); the isolation check passes again.
9. **The stops, lying down then standing** (first-boot.md §3), with DimOS connected:
   `dimos run --local-relay unitree-go2 houndkeeper.battery houndkeeper.phone-cockpit --extensions-dir cockpit/extensions`
   (plus the robot's address, from `real/.env`).
10. **DimOS drives him** (first-boot.md §4): stand, walk, turn, lie down from the phone.

## Calibration (about two and a half hours, real/calibration/README.md "Day one, in order")

**The five first, in this order** ([GWP]): what the sim's launch numbers lean on most (a sensitivity study,
docs/research/sensitivity-2026-10-13.md). Each but the compression (judged by eye) prints `day one ([GWP]): pass | outside | fails`; inside the band the
false alarms stayed within the replays' spread, `fails` is where the study saw them rise, between is untested and
counts as outside. What to do when outside: the README's day one, same number.

1. **Mount** (head_camera.md 1-3 now, 20 min; "Whether it stays put" after nights 1 and 2): `measure.py mount` on
   each night's check. Pass: within 0.1 deg pitch (0.3 yaw, 5 mm) of the calibration and of night 1. Fails at 0.5
   deg (19.0 false alarms a night against 11.2).
2. **Frame lag** (timing.md 1-3, 15 min): `measure.py lag <db> --expect <stored lag>`. Pass: within 20 ms, and
   night 2 within 20 ms of night 1. Fails at 50 ms (17.5).
3. **Odometry drift** (odometry.md 1-2, 30 min; the square again before night 2): `measure.py odom <db> --truth
   real/local/taped_square.csv`. Pass: at or under the file's estimate. Fails at twice it (15.8).
4. **Compression on the darkest floor** (camera.md 3, 15 min): `measure.py compression <db> --sim <sim dark floor
   night> --sheet real/local/compression.png`. Pass, by eye on the picture: no flatter or blockier than the sim's
   frames at q50. Fails at q30 (15.3). Check a carpet too: q30 end to end lost every carpet toy on the GPU (the
   sensitivity doc's "On the GPU" section).
5. **Noise** (camera.md 1, 5 min, recorded with 4 and read before it): `measure.py noise <db> --sim <sim db>`. Pass:
   up to 4 times the sim's. Nothing larger tested.

Then the rest of the README's list (weight and pose, the lens's place, the lidar's wall, the battery walk).

11. **Record every procedure as a session**, so each recording is cataloged with its purpose and the code and
    parameters in force ([HqQ]):

        hk-data record --real --purpose "day one: still minute" --tag calibration --tag day-one --firmware <v> -- \
            run unitree-go2 houndkeeper.phone-cockpit --extensions-dir cockpit/extensions

    The session refuses to start without a fresh passing isolation check or under 50 GB free, adds the battery
    stream, and catalogs the recording when the run ends (moved into `data/recordings/real/`, unscreened).
12. **Run the procedures from the phone's Procedures page** ([vRJ]): each shows its steps and live numbers, then the
    result beside the file's value; "Write to params" only after reading it. The hands-on ones (tape, scale,
    stopwatch) are in `real/calibration/*.md`.
12a. **What the robot's IMU gives us** ([fxE] [1sW]): record a still minute and a turning minute, then list what the low-state stream carries (orientation only, or raw gyro and accelerometer) and at what rate; it decides how the lag and mount are calibrated (build-or-depend [eK3] entry 19).
13. **The gap report** ([KbT]): `$PY real/calibration/measure.py gap "$(hk-data path --numbers-only <id>)"` (the reference
    defaults to `real/calibration/sim_reference.json`) sets each real number beside the sim's and says which
    parameter to move.

## The evening

14. **A first night in the house**: a recorded session with `--purpose "first night"`, then the night pipeline on
    the real recording (the replay path, sim-to-real-calibration [CCS]).
14a. **The second night checks the first** ([GWP]): after night 2, the mount night to night (`measure.py mount`),
    the lag on night 2's turns and the taped square again; the launch numbers are trusted on the robot only when
    mount, lag and drift pass on both nights.
15. **Look at what he tried to call**: `real/local/logs/robot-dns.log` and `router.log` (every name and address),
    into the first-boot record (`real/local/first-boot-record.md`).
16. **Keep it**: `hk-data verify`; `hk-data sync` once the server is up (until then real recordings are single-copy,
    [Zw9]); screen any recording before its frames are used: `tools/screen_people.py`, then
    `hk-data screened <id> clear|people` ([R1X]).
17. **Re-run what depends on what changed** ([7Xn]): every parameter written today names the sims to re-run in
    `real/calibration/README.md`.

## The phone cockpit over Tailscale ([rhP], [4uq])

Once the tailnet's HTTPS certificates are on and the laptop's user may fetch them
(`sudo tailscale set --operator=$USER`, once):

    tailscale cert --cert-file real/local/tls/cockpit.crt --key-file real/local/tls/cockpit.key <laptop>.<tailnet>.ts.net
    real/bringup/make_relay_auth.sh      # robot keys (barrymore, project) and the phone's viewer token: real/local/relay-auth.json, mode 600
    deno run -A ../dimos/web/relay/main.ts --host <the laptop's Tailscale address> --port 7780 \
        --cert real/local/tls/cockpit.crt --key real/local/tls/cockpit.key --auth-file real/local/relay-auth.json \
        --cockpit-dir ../dimos/web/cockpit/dist --sdk-dir ../dimos/web/sdk/dist \
        --extensions-dir cockpit/extensions
    RELAY_KEY="$(jq -r '.robots.barrymore' real/local/relay-auth.json)" \
        dimos --robot-id barrymore run --relay-url https://<laptop>.<tailnet>.ts.net:7780 unitree-go2 houndkeeper.phone-cockpit

The robot id must match the auth file's `robots` entry its key is bound to (DimOS's docs/web/relay.md "Auth file");
the key comes from the file at run time, never from a committed `.env`.

The phone opens `https://<laptop>.<tailnet>.ts.net:7780/` and enters the viewer token once. Certificates last 90
days: renew with the same `tailscale cert` line.

### The project pages, always on (project-app [eTc], [RzM])

The relay and the project service run as systemd user units, so the Last night page is there with no robot or sim
running; the robot, when it runs, registers beside "project" and the Cockpit's robot picker switches between them.
Started this way, the relay replaces the hand-run `deno` line above. Once the certificate and auth file exist:

    sed -e "s|@REPO@|$PWD|g" -e "s|@PYTHON@|$(cd ../dimos && pwd)/.venv/bin/python|g" \
        -e "s|@DENO@|$(command -v deno)|g" real/bringup/hk-relay.service > ~/.config/systemd/user/hk-relay.service
    sed -e "s|@REPO@|$PWD|g" -e "s|@PYTHON@|$(cd ../dimos && pwd)/.venv/bin/python|g" \
        real/bringup/hk-project.service > ~/.config/systemd/user/hk-project.service
    systemctl --user daemon-reload && systemctl --user enable --now hk-relay hk-project
    loginctl enable-linger "$USER"        # keeps them up with no one logged in

Both refuse any address but the Tailscale one ([Dlx]). To try it on the laptop alone first, without certificates:
`python -m houndkeeper.project_stack relay --loopback` and, in a second shell,
`python -m houndkeeper.project_stack project --loopback`, then `http://127.0.0.1:7780/` in a browser.
