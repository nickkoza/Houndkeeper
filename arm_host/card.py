"""A blank SD card made into an arm host in one step ([OmH]): which card may be written ([ObX]), the profiles ([H9j],
decision [tDE]) and the first-boot files cloud-init reads from the boot partition (decision [mX4]). Written as JSON,
which is valid YAML, so nothing a password or a name contains can break the quoting. Used by flash_card.sh:
  python3 -m arm_host.card device --lsblk <lsblk -J -b output file> --name /dev/sdX   (prints its identity first)
  python3 -m arm_host.card files --out <dir> --profile houndkeeper --ssh-key-file ~/.ssh/id_ed25519.pub ..."""
import argparse, ipaddress, json, re, sys, uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
MAX_CARD_BYTES = 256 * 1000 ** 3          # bigger than any card we'd use: a drive, not a card
SYSTEM_MOUNTS = ("/", "/boot", "/boot/efi", "/home", "/usr", "/var", "/etc", "[SWAP]")
DESKTOP_MOUNTS = ("/media/", "/run/media/")    # where a desktop mounts a card put in; anything else is someone's disk


class Refused(Exception):
    pass


def _tree(d):
    yield d
    for c in d.get("children") or []: yield from _tree(c)


def choose_device(lsblk, name):
    """the disk named, if it is safe to erase: a removable card (or the laptop's own card slot), not too big, nothing of
    it in use: no file system mounted anywhere but the desktop's mount places, no encrypted, LVM or RAID volume open"""
    disks = [d for d in lsblk.get("blockdevices", []) if d.get("type") == "disk"]
    d = next((d for d in disks if d.get("path") == name or f"/dev/{d.get('name')}" == name), None)
    if d is None: raise Refused(f"{name}: no such disk")
    slot = bool(re.fullmatch(r"mmcblk\d+", str(d.get("name", ""))))   # the laptop's card slot (not eMMC boot areas)
    if not (bool(d.get("rm")) or slot) or d.get("tran") in ("nvme", "sata", "ata", "sas"):  # a USB hard drive is rm 0
        raise Refused(f"{name}: not a removable card")
    if int(d.get("size") or 0) > MAX_CARD_BYTES: raise Refused(f"{name}: {int(d['size']) / 1e9:.0f} GB is too big for a card")
    parts = list(_tree(d))
    held = [x.get("name") for x in parts if x.get("type") in ("crypt", "lvm", "dm", "md", "raid0", "raid1", "raid4",
                                                                "raid5", "raid6", "raid10", "mpath")]
    if held: raise Refused(f"{name}: the laptop is using it ({', '.join(map(str, held))} open)")
    mounts = [m for x in parts for m in (x.get("mountpoints") or []) if m]
    bad = [m for m in mounts if m in SYSTEM_MOUNTS or not m.startswith(DESKTOP_MOUNTS)]
    if bad: raise Refused(f"{name}: the laptop is using it ({', '.join(bad)})")
    return d


def identity(d):
    """what the user confirmed, re-checked just before the write: the same path, serial, model and size"""
    return f"{d.get('path') or '/dev/' + str(d.get('name'))}|{d.get('serial') or ''}|{d.get('model') or ''}|{int(d.get('size') or 0)}"


def _check_wifi(country, psk, ssid=""):
    if any(ord(ch) > 0xFFFF for ch in ssid):            # cloud-init's YAML reader breaks JSON's surrogate-pair escapes
        raise Refused("the network name has a character (an emoji?) the Pi's first boot would misread")
    if not all(32 <= ord(ch) <= 126 for ch in psk): raise Refused("the Wi-Fi password: printable ASCII only (WPA)")
    if not re.fullmatch(r"[A-Z]{2}", country or ""): raise Refused(f"Wi-Fi country {country!r}: two capital letters")
    if not (8 <= len(psk) <= 63 or re.fullmatch(r"[0-9a-fA-F]{64}", psk)):
        raise Refused("the Wi-Fi password: 8 to 63 characters, or a 64-digit hex key")


def profiles():
    return {p.stem: json.loads(p.read_text()) for p in sorted((HERE / "profiles").glob("*.json"))}


def host_settings(profile, laptop_ip, extra=None):
    """the profile's settings, and its services: the report gives each one's state, first boot enables each ([4WJ], [BRx])"""
    p = profiles()[profile]
    return dict(p["host"], profile=profile, laptop_ip=laptop_ip, services=list(p["services"]), **(extra or {}))


def timesyncd_conf(time_server):
    """the one clock ([aVJ], decision [LSY]): the source given, the router the laptop also follows; never a fallback on
    the internet ([DzZ]), so with none given the clock is simply not set"""
    if time_server:
        try: ipaddress.ip_address(time_server)
        except ValueError: raise Refused(f"time source {time_server!r}: one address") from None
    return f"[Time]\nNTP={time_server or ''}\nFallbackNTP=\n"


def first_boot_files(profile, ssh_key, wifi, laptop_ip, release, country="US", hostname="hk-arm", offline_packages=False,
                     speaker_mac="", microphone=True, time_server=""):
    """the files for the card's boot partition: cloud-init's three, and hk/ (the first-boot script reads them).
    microphone=False for a card on any network but the robot's ([gFa]): the microphone is not streamed over it."""
    if profile not in profiles(): raise Refused(f"no profile {profile}")
    ssid, psk = wifi; _check_wifi(country, psk, ssid)
    host = host_settings(profile, laptop_ip, {"speaker_mac": speaker_mac})
    if not microphone: host["features"] = dict(host.get("features", {}), ears=False)
    user_data = {
        "hostname": hostname, "manage_etc_hosts": True,
        "users": [{"name": "hk", "groups": "sudo,spi,gpio,audio,bluetooth,video", "shell": "/bin/bash",
                   "sudo": "ALL=(ALL) NOPASSWD:ALL", "lock_passwd": True, "ssh_authorized_keys": [ssh_key.strip()]}],
        "ssh_pwauth": False, "disable_root": True,
        "bootcmd": [["raspi-config", "nonint", "do_wifi_country", country]],     # the Wi-Fi radio stays off until set
        "runcmd": [["bash", "/boot/firmware/hk/firstboot.sh"]]}
    network = {"network": {"version": 2, "wifis": {"wlan0": {"dhcp4": True, "optional": True, "regulatory-domain": country,
                                                             "access-points": {ssid: {"password": psk}}}}}}
    meta = {"instance-id": f"hk-{uuid.uuid4()}", "dsmode": "local"}           # new on every card: first boot runs again
    card = {"profile": profile, "release": release, "offline_packages": bool(offline_packages)}
    return {"user-data": "#cloud-config\n" + json.dumps(user_data, indent=1), "network-config": json.dumps(network, indent=1),
            "meta-data": json.dumps(meta), "hk/host.json": json.dumps(host, indent=1),
            "hk/card.json": json.dumps(card), "hk/timesyncd.conf": timesyncd_conf(time_server)}


def main():
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("device"); d.add_argument("--lsblk", required=True); d.add_argument("--name", required=True)
    f = sub.add_parser("files"); f.add_argument("--out", required=True); f.add_argument("--profile", required=True)
    f.add_argument("--ssh-key-file", required=True); f.add_argument("--ssid", required=True); f.add_argument("--psk-file", required=True)
    f.add_argument("--laptop-ip", required=True); f.add_argument("--release", required=True); f.add_argument("--country", default="US")
    f.add_argument("--offline", action="store_true"); f.add_argument("--speaker-mac", default="")
    f.add_argument("--no-microphone", action="store_true"); f.add_argument("--time-server", default="")
    h = sub.add_parser("host"); h.add_argument("--profile", required=True); h.add_argument("--laptop-ip", required=True)
    h.add_argument("--speaker-mac", default="")
    a = ap.parse_args()
    try:
        if a.cmd == "device":
            dev = choose_device(json.load(open(a.lsblk)), a.name)
            mounts = [m for x in _tree(dev) for m in (x.get("mountpoints") or []) if m]
            print(f"{identity(dev)}\t{dev['path']}\t{dev.get('model') or ''}\t{int(dev['size']) / 1e9:.1f} GB"
                  f"{chr(9) + 'mounted at ' + ', '.join(mounts) if mounts else ''}")
        elif a.cmd == "files":
            files = first_boot_files(a.profile, Path(a.ssh_key_file).read_text(), (a.ssid, Path(a.psk_file).read_text().strip()),
                                     a.laptop_ip, a.release, a.country, offline_packages=a.offline, speaker_mac=a.speaker_mac,
                                     microphone=not a.no_microphone, time_server=a.time_server)
            for name, text in files.items():
                p = Path(a.out) / name; p.parent.mkdir(parents=True, exist_ok=True); p.write_text(text)
        else:
            print(json.dumps(host_settings(a.profile, a.laptop_ip, {"speaker_mac": a.speaker_mac}), indent=1))
    except Refused as e:
        print(f"refused: {e}", file=sys.stderr); sys.exit(3)


if __name__ == "__main__":
    main()
