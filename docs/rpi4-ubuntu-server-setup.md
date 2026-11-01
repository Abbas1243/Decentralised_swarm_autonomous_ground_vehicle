## Booting Ubuntu Server on Raspberry Pi 4B

Tested with Ubuntu Server 24.04 LTS (64-bit) on a Raspberry Pi 4B, booting from microSD.

**Requirements:** microSD card (16 GB+, Class 10/A1), card reader, 5V 3A USB-C power supply, Ethernet cable (recommended for first boot).

---

### Step 1: Flash Ubuntu Server to the microSD card

**Option A (GUI):** Use Raspberry Pi Imager: *Raspberry Pi 4 → Other general-purpose OS → Ubuntu → Ubuntu Server 24.04 LTS (64-bit)*. Use **Edit Settings** to set hostname, user, SSH and Wi-Fi, then write.

**Option B (CLI, Linux/macOS):**

```bash
# Replace the filename with the latest image from https://ubuntu.com/download/raspberry-pi
IMG=ubuntu-24.04-preinstalled-server-arm64+raspi.img.xz

# Identify your SD card (e.g. /dev/sdX or /dev/mmcblk0). Double-check this!
lsblk

# Flash (this ERASES the target device)
xzcat "$IMG" | sudo dd of=/dev/sdX bs=4M status=progress conv=fsync
sync
```

### Step 2: Preconfigure headless access (cloud-init)

After flashing, mount the SD card's `system-boot` partition and edit `user-data` and `network-config`. Skip this if you already configured everything in Imager.

```yaml
# system-boot/user-data
#cloud-config
hostname: rpi4
manage_etc_hosts: true
timezone: Asia/Kolkata
ssh_pwauth: true
users:
  - name: ubuntu
    groups: [sudo]
    shell: /bin/bash
    sudo: ALL=(ALL) NOPASSWD:ALL
    lock_passwd: false
    plain_text_passwd: "change-me-now"
```

```yaml
# system-boot/network-config (Wi-Fi example; not needed for Ethernet)
version: 2
wifis:
  wlan0:
    dhcp4: true
    optional: true
    access-points:
      "YOUR_WIFI_SSID":
        password: "YOUR_WIFI_PASSWORD"
```

### Step 3: First boot

Insert the card, connect Ethernet, then power on. The first boot runs cloud-init and takes 2 to 5 minutes. Do not power-cycle it.

```bash
# On a monitor/serial console, or after SSH, confirm cloud-init has finished
cloud-init status --wait
```

### Step 4: Find the Pi and connect over SSH

```bash
# Try mDNS first
ssh ubuntu@rpi4.local

# If that fails, scan your subnet for the Pi's IP
nmap -sn 192.168.1.0/24

# Then connect using the IP
ssh ubuntu@<PI_IP_ADDRESS>
```

### Step 5: Update the system and verify

```bash
sudo apt update && sudo apt upgrade -y
sudo reboot
```

After the reboot, SSH back in and verify:

```bash
lsb_release -a          # Should show Ubuntu 24.04 LTS
uname -m                # Should print aarch64
hostname -I             # Shows the Pi's IP address
vcgencmd measure_temp   # Optional: CPU temperature (needs libraspberrypi-bin)
```

---

### Troubleshooting

| Problem | Fix |
|---|---|
| No SSH connection | Confirm SSH was enabled in Imager or `user-data`, and that the Pi is on the same network. |
| No Wi-Fi on first boot | Recheck `network-config` indentation and credentials, or use Ethernet. |
| `apt` lock errors after boot | `unattended-upgrades` is running. Wait a few minutes and retry. |
| Boot hangs / no ACT LED activity | Reflash the card and try a different SD card or power supply. |

> **Security note:** Change the default password immediately after first login (`passwd`) and prefer SSH key authentication for anything beyond testing.
