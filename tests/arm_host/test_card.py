"""The arm host's card: one step to a working host [OmH], profiles [H9j], and only ever the named removable card
written [ObX]."""
import json

import pytest

from arm_host import card

GB = 1000 ** 3


def lsblk(*disks):
    return {"blockdevices": list(disks)}


def disk(name, size_gb, rm, tran, mounts=(), parts=()):
    return {"name": name, "path": f"/dev/{name}", "size": int(size_gb * GB), "rm": rm, "tran": tran, "type": "disk",
            "model": "Test", "mountpoints": list(mounts) or [None],
            "children": [{"name": f"{name}{i+1}", "path": f"/dev/{name}{i+1}", "type": "part", "mountpoints": [m]}
                         for i, m in enumerate(parts)]}


SYSTEM = disk("nvme0n1", 1000, False, "nvme", parts=("/boot/efi", "/"))
CARD = disk("sdb", 32, True, "usb", parts=(None, None))
MMC = disk("mmcblk0", 64, True, None, parts=())


def test_removable_card_accepted():
    assert card.choose_device(lsblk(SYSTEM, CARD), "/dev/sdb")["path"] == "/dev/sdb"
    assert card.choose_device(lsblk(SYSTEM, MMC), "/dev/mmcblk0")["path"] == "/dev/mmcblk0"


def test_system_disk_refused():
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, CARD), "/dev/nvme0n1")


def test_fixed_disk_refused():
    fixed = disk("sdc", 32, False, "sata")
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, fixed), "/dev/sdc")


def test_mounted_system_partition_refused():
    usb_root = disk("sdd", 32, True, "usb", parts=("/home",))
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, usb_root), "/dev/sdd")


def test_too_large_refused():
    big = disk("sde", 2000, True, "usb")
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, big), "/dev/sde")


def test_unknown_device_refused():
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, CARD), "/dev/sdz")


def test_profiles():
    ps = card.profiles()
    assert "houndkeeper" in ps
    for name, p in ps.items():
        assert p["services"] and "host" in p
    assert ps["houndkeeper"]["host"]["features"]["arm"] and ps["houndkeeper"]["host"]["features"]["voice"]


def first_boot(**kw):
    args = dict(profile="houndkeeper", ssh_key="ssh-ed25519 AAAATEST laptop", wifi=("HouseNet", "pa ss\"word"),
                laptop_ip="192.0.2.10", release="12-abc", country="US", hostname="hk-arm", offline_packages=True)
    args.update(kw)
    return card.first_boot_files(**args)


def load(text):
    assert text.startswith("#cloud-config\n") or not text.startswith("#")
    return json.loads(text.split("\n", 1)[1] if text.startswith("#") else text)


def test_user_data_key_only():
    ud = load(first_boot()["user-data"])
    u = ud["users"][0]
    assert u["name"] == "hk" and u["lock_passwd"] is True and "ssh-ed25519 AAAATEST laptop" in u["ssh_authorized_keys"]
    assert ud["ssh_pwauth"] is False
    assert any("firstboot.sh" in " ".join(c) if isinstance(c, list) else "firstboot.sh" in c for c in ud["runcmd"])


def test_network_config_wifi():
    nc = load(first_boot()["network-config"])
    ap = nc["network"]["wifis"]["wlan0"]["access-points"]
    assert ap["HouseNet"]["password"] == "pa ss\"word"


def test_instance_id_new_each_card():
    a, b = load(first_boot()["meta-data"]), load(first_boot()["meta-data"])
    assert a["instance-id"] != b["instance-id"]


def test_host_settings_carry_the_profile():
    h = json.loads(first_boot()["hk/host.json"])
    assert h["profile"] == "houndkeeper" and h["laptop_ip"] == "192.0.2.10" and h["features"]["arm"]


def test_offline_flag_reaches_first_boot():
    assert json.loads(first_boot()["hk/card.json"])["offline_packages"] is True
    assert json.loads(first_boot(offline_packages=False)["hk/card.json"])["offline_packages"] is False


# ---- red team, 3 Oct 2026 (docs/research/redteam-arm-host-2026-10-03.md) -----------------------------------------------

def test_a_usb_hard_drive_is_not_a_card():
    """[ObX] tran usb alone passed as 'removable': an external USB SSD (rm 0) of 256 GB or less, a backup or a boot
    drive, could be named and erased"""
    ssd = disk("sdf", 240, False, "usb", parts=("/media/nick/Backup",))
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, ssd), "/dev/sdf")
    ssd_unmounted = disk("sdg", 240, False, "usb")
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, ssd_unmounted), "/dev/sdg")


def test_a_mount_deeper_than_a_partition_is_seen():
    """[ObX] only the disk and its partitions were looked at: an encrypted or LVM disk mounts its file systems one
    level further down, so a removable drive holding the laptop's /home looked unused"""
    d = disk("sdh", 64, True, "usb", parts=(None,))
    d["children"][0]["children"] = [{"name": "luks-home", "path": "/dev/mapper/luks-home", "type": "crypt",
                                     "mountpoints": ["/home"]}]
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, d), "/dev/sdh")


def test_an_opened_encrypted_or_lvm_volume_is_in_use_even_unmounted():
    d = disk("sdi", 64, True, "usb", parts=(None,))
    d["children"][0]["children"] = [{"name": "vg-data", "path": "/dev/mapper/vg-data", "type": "lvm", "mountpoints": [None]}]
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, d), "/dev/sdi")


def test_a_card_mounted_anywhere_but_the_desktop_mount_places_is_refused():
    """[ObX] a stick mounted at /mnt or /srv is someone's working disk; a fresh card auto-mounts under /media"""
    for m in ("/mnt/spinning-drive", "/srv/data", "/opt/x", "/data"):
        d = disk("sdj", 32, True, "usb", parts=(m,))
        with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, d), "/dev/sdj")
    ok = disk("sdk", 32, True, "usb", parts=("/media/nick/bootfs", "/media/nick/rootfs"))
    assert card.choose_device(lsblk(SYSTEM, ok), "/dev/sdk")


def test_emmc_boot_areas_are_not_cards():
    boot = {"name": "mmcblk0boot0", "path": "/dev/mmcblk0boot0", "size": 4 * 1024 ** 2, "rm": False, "tran": None,
            "type": "disk", "mountpoints": [None]}
    with pytest.raises(card.Refused): card.choose_device(lsblk(SYSTEM, boot), "/dev/mmcblk0boot0")


def test_identity_is_reported_for_the_recheck():
    """[ObX] the identity the user confirmed is what the write re-checks just before it starts"""
    c = dict(CARD, serial="ABC123")
    assert card.identity(card.choose_device(lsblk(SYSTEM, c), "/dev/sdb")) == "/dev/sdb|ABC123|Test|%d" % (32 * GB)


@pytest.mark.parametrize("country", ["U", "USA", "us; reboot", "", "1Z"])
def test_a_bad_country_is_refused(country):
    with pytest.raises(card.Refused): first_boot(country=country)


@pytest.mark.parametrize("psk", ["short", "x" * 64 + "y", "g" * 64])
def test_a_wifi_password_wpa_cannot_take_is_refused(psk):
    """a password WPA refuses leaves a headless Pi with no network and no other way in: refused on the laptop"""
    with pytest.raises(card.Refused): first_boot(wifi=("HouseNet", psk))
    assert first_boot(wifi=("HouseNet", "a" * 64))           # 64 hex digits: a raw key


def test_a_card_for_another_network_sends_no_microphone(monkeypatch):
    """[gFa] a card for the house network (the first, online card) must not stream the microphone over it: the
    laptop's robot-network address there may be anyone's (on a profile with ears; houndkeeper's has none)"""
    real = card.profiles()
    eared = json.loads(json.dumps(real["houndkeeper"])); eared["host"]["features"]["ears"] = True
    monkeypatch.setattr(card, "profiles", lambda: {**real, "eared": eared})
    h = json.loads(first_boot(profile="eared", microphone=False)["hk/host.json"])
    assert h["features"]["ears"] is False
    assert json.loads(first_boot(profile="eared")["hk/host.json"])["features"]["ears"] is True


def test_what_cloud_init_would_misread_is_refused():
    """[OmH] JSON is YAML only up to a point: cloud-init's YAML reader turns a JSON-escaped emoji into two broken
    characters, so an SSID with one would never be joined. A WPA passphrase is printable ASCII by the standard."""
    with pytest.raises(card.Refused): first_boot(wifi=("Pumpkin \U0001F383", "password1"))
    with pytest.raises(card.Refused): first_boot(wifi=("HouseNet", "pässwörd1"))
    assert first_boot(wifi=("Café", "password1"))
