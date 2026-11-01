## Wi-Fi setup on Raspberry Pi 4B (Ubuntu Server, Netplan + systemd-networkd)

Connect the Pi to a home Wi-Fi and a mobile hotspot using Netplan only (no `nmcli` needed). Netplan generates the config for `systemd-networkd` and `wpa_supplicant`, which automatically joins whichever listed network is in range.

> **Safety tip:** Do this over Ethernet or a monitor/keyboard if possible. If you are SSH'd in over `wlan0`, a bad config can lock you out.

---

### Step 1: Inspect and back up the existing configuration

```bash
# See the merged config Netplan is actually using
sudo netplan get

# List config files and check which one already defines wlan0
ls -l /etc/netplan/

# Back up the cloud-init generated file
sudo cp /etc/netplan/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml.backup
```

### Step 2: Define all Wi-Fi networks in one file

Netplan merges every file in `/etc/netplan/`, and a later file can override or extend an earlier one. To avoid surprises with `wlan0` defined in two places, remove the Wi-Fi block from `50-cloud-init.yaml` and keep all Wi-Fi networks together in one new file.

```bash
sudo nano /etc/netplan/60-wifi.yaml
```

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      regulatory-domain: IN
      access-points:
        "HOME_WIFI_SSID":
          password: "HOME_WIFI_PASSWORD"
        "MOBILE_HOTSPOT_SSID":
          password: "MOBILE_HOTSPOT_PASSWORD"
```

Netplan requires restrictive permissions on files containing passwords:

```bash
sudo chmod 600 /etc/netplan/60-wifi.yaml
```

> Cloud-init can regenerate `50-cloud-init.yaml` on boot. To stop it, create `/etc/cloud/cloud.cfg.d/99-disable-network-config.cfg` containing `network: {config: disabled}`.

### Step 3: Validate the configuration

```bash
sudo netplan generate
```

No output means the YAML is valid. Fix any indentation or duplicate-key errors before continuing.

### Step 4: Apply safely

```bash
sudo netplan try
```

Press **Enter** to accept within the timeout. If you lose connectivity and don't confirm, Netplan rolls back automatically.

### Step 5: Verify and connect

```bash
ip addr show wlan0          # Confirm an IP was assigned
iw dev wlan0 link           # Shows which SSID you are connected to
ping -c 3 8.8.8.8           # Confirm internet access
```

Then, from your laptop on the same network:

```bash
ssh ubuntu@<PI_IP_ADDRESS>
```

Typical hotspot subnets: Android is usually `192.168.43.x`, iPhone is `172.20.10.x`.

---

### Troubleshooting

| Problem | Fix |
|---|---|
| `Permissions ... are too open` warning | `sudo chmod 600 /etc/netplan/*.yaml` |
| Wi-Fi does not connect | Check SSID/password. Many hotspots must be set to **2.4 GHz** (or "maximize compatibility" on iPhone). |
| Config reverts after reboot | Cloud-init regenerated `50-cloud-init.yaml`. Disable its network config (see Step 2). |
| Debug connection | `journalctl -u wpa_supplicant -n 50` and `sudo networkctl status wlan0` |
