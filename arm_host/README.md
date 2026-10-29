# Arm host (the Raspberry Pi on the robot)

The robot's voice, ears and eyes, and later the arm and the wrist camera. Spec: the arm host, [JIY].

## A clean baseline: a new card ([OmH])

Put an SD card in the laptop, then:

    lsblk                                     # find the card, e.g. /dev/sdb
    set -a; . real/.env; set +a               # the house's network names and addresses (gitignored)
    arm_host/flash_card.sh /dev/sdb --profile houndkeeper --laptop-ip $LAPTOP_IP [--speaker-mac AA:BB:CC:DD:EE:FF]

It checks the card is a removable one and nothing the laptop is using, shows it, and asks you to type its name. It
writes Raspberry Pi OS (downloaded once, kept by checksum in ~/.cache/houndkeeper), joins the robot network (named in `real/.env`), puts in
your SSH key and the current release. Put the card in the Pi and power it: about five minutes later it is a working
host at hk@hk-arm.local. Put the Bluetooth speaker in pairing mode for the first boot; after that it reconnects.

The Pi never goes online. Its packages ride on every card: `flash_card.sh` fetches them on the laptop the first time
(`arm_host/fetch_packages.sh`: the image's own apt sources, installed packages and archive keys, read out of the
checked image, so every package is verified as the Pi's own apt would), then reuses them. Run it again to pick up
security updates. A card is never made without them, and a card's first boot stops rather than reach for the
internet.

## Switching profile ([H9j])

    arm_host/switch_profile.sh hk-arm.local houndkeeper --laptop-ip $LAPTOP_IP     # seconds, no reflash

Or flash a card with that `--profile` for a clean one. Profiles: arm_host/profiles/*.json (today only houndkeeper).

## A quick patch ([871])

    arm_host/deploy.sh hk-arm.local --robot-still

Pushes this commit's arm_host/ as a numbered release; it rolls itself back if it fails its check.

## Checking a host on the Pi ([JIY], decision [Uyb])

    arm_host/pi_check.sh hk-arm.local [--kill-test] [--only UYK,27n]

The Pi's half runs over SSH, as root, from the laptop's own copy of the check (never the Pi's installed release), then the laptop's (ports, a password login, the clock, the
reports; stop DimOS's listener on the report port first). Each line is pass, fail, not built, manual (a step for you)
or measured; the record goes to real/local/arm-host-checks/. Two cards from one commit:
`python3 -m arm_host.pi_check compare <record a> <record b>`. From a machine on the guest network:
`arm_host/pi_check.sh <pi address> --expect-unreachable`. After each power cut: `--only 27n`.

The packages a card carries are pinned in `pi-packages.lock.json` ([UYK]): `fetch_packages.sh` rewrites it (commit
it), and `flash_card.sh` refuses packages it does not name.
