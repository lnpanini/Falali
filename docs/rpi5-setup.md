# Raspberry Pi 5 setup runbook — TrolleyBot main controller

Companion to [`docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md`](superpowers/specs/2026-08-04-rpi5-main-controller-design.md).
Follow in order. Steps 1–3 are the ones that bite people.

---

## 1. Power — get this wrong and you'll chase ghost bugs for a week

The Pi 5 needs **5 V at 5 A (27 W)**. With anything less it boots, runs, and then browns out the
moment two USB devices draw current at once — which presents as random ESP disconnects, not as a
power warning.

On a robot you are not using a wall adapter, so:

- Feed the Pi from a **dedicated 5 V/5 A buck converter** off the main battery, into the USB-C port
  or the 5 V rail on the 40-pin header (header input bypasses the Pi's input protection — use the
  USB-C port unless you have a reason not to).
- Add **bulk capacitance** (≥ 1000 µF) at the Pi's input. Motor inrush on a shared battery drags the
  rail down for milliseconds; the Pi notices.
- The motor supply and the logic supply share **ground only**. Never share the 5 V rail with motor
  drivers.

Enable full USB current budget (otherwise the Pi caps total USB draw at 600 mA):

```bash
sudo sed -i '$a usb_max_current_enable=1' /boot/firmware/config.txt
```

Verify after reboot — if this reports a low limit, your PSU is not being recognised as 5 A capable:

```bash
vcgencmd get_config usb_max_current_enable
dmesg | grep -i "power\|undervolt"
```

**Powering the ESPs:** USB from the Pi is fine for logic (an ESP32 devkit peaks ~250 mA with WiFi
off, and you should turn WiFi off). What is *not* fine is the ESP's 5 V pin also feeding motor
driver logic — that couples motor noise into the Pi's USB rail.

---

## 2. OS and base install

Raspberry Pi OS (64-bit) Lite. No desktop — you want the CPU and the determinism.

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y python3-venv python3-pip git tmux
sudo systemctl disable --now bluetooth ModemManager   # ModemManager grabs serial ports. Really.
```

`ModemManager` probing a freshly-plugged ESP32 will send AT commands at it and can hold the port for
seconds. Disabling it is not optional.

**Boot from NVMe if you can.** Continuous telemetry logging kills SD cards, and a robot that gets
power-cycled abruptly will eventually corrupt one. A PCIe HAT + small NVMe drive is the single best
reliability upgrade available here.

---

## 3. Stable device names for the two ESPs

`/dev/ttyUSB0` and `/dev/ttyUSB1` are assigned in **enumeration order**, which changes between boots.
If you hardcode them you will one day command the arm with wheel setpoints. Bind by hardware serial
number instead.

Plug in **one** ESP and read its identity:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -E 'ATTRS\{(idVendor|idProduct|serial)\}' | head -5
```

Repeat for the other. Then:

```bash
sudo tee /etc/udev/rules.d/99-trolleybot.rules >/dev/null <<'EOF'
# Match on the USB-serial adapter's unique serial number, not the port order.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="<BASE_SERIAL>", SYMLINK+="ttyTB_BASE", MODE="0660", GROUP="dialout"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="<ARM_SERIAL>",  SYMLINK+="ttyTB_ARM",  MODE="0660", GROUP="dialout"
EOF

sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER   # log out and back in
```

Confirm:

```bash
ls -l /dev/ttyTB_*
```

> **If your ESP32-S3 uses native USB-CDC** it enumerates as `/dev/ttyACM*` with vendor `303a`, and
> some devkits ship with an **identical serial string on every unit** — in which case
> `ATTRS{serial}` cannot disambiguate them. Two ways out: set a unique USB serial in firmware, or
> match on physical USB port path (`KERNELS=="1-1.2"`) and commit to never moving the cables. Prefer
> the firmware fix; port-path matching is a trap you will forget about.

---

## 4. Python environment

```bash
mkdir -p ~/trolleybot && cd ~/trolleybot
python3 -m venv .venv
source .venv/bin/activate
pip install pyserial-asyncio-fast numpy scipy
```

`pyserial-asyncio-fast` rather than plain `pyserial` — the async version avoids a blocking read
thread per port, which matters when you are servicing two links inside a 20 ms budget.

---

## 5. Latency tuning

Two settings that dominate USB-serial jitter:

```bash
# Reduce the FTDI/CH34x driver's read latency timer from 16 ms to 1 ms.
# 16 ms is longer than your entire control period.
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Make it permanent by adding to the udev rules above:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{serial}=="<BASE_SERIAL>", SYMLINK+="ttyTB_BASE", RUN+="/bin/sh -c 'echo 1 > /sys/bus/usb-serial/devices/%k/latency_timer'"
```

And give the control loop scheduling priority — but **only** the loop, never the logger:

```bash
sudo setcap cap_sys_nice+ep $(readlink -f ~/trolleybot/.venv/bin/python3)
```

Then in the bridge, request `SCHED_FIFO` at a modest priority (see `pi/bridge.py`).

> Real-time priority on a non-realtime kernel reduces jitter; it does not eliminate it. This is
> exactly why the ESP-side watchdog exists and why no safety property may depend on the Pi being
> on time. Tune for good behaviour, design for bad.

---

## 6. Run as a service

```bash
sudo tee /etc/systemd/system/trolleybot.service >/dev/null <<'EOF'
[Unit]
Description=TrolleyBot Pi bridge
After=multi-user.target

[Service]
Type=simple
User=bryan
WorkingDirectory=/home/bryan/trolleybot
ExecStart=/home/bryan/trolleybot/.venv/bin/python -m pi.bridge
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now trolleybot
journalctl -u trolleybot -f
```

Do **not** enable this until step 1 of the migration order (watchdog) is proven. An auto-restarting
service that commands motors on boot is how robots drive into walls unattended.

---

## 7. Development workflow

Keep flashing the ESPs from your Mac with PlatformIO as you do now — the Pi is not a build machine.
Mount the repo on the Pi over SSHFS, or just `git pull` on the Pi and edit on the Mac.

```bash
# From the Mac, watch the robot:
ssh bryan@trolleybot.local 'journalctl -u trolleybot -f'
```

---

## Bring-up checklist

Tick these before letting the Pi command anything:

- [ ] `vcgencmd get_throttled` returns `0x0` under motor load (no undervoltage)
- [ ] `/dev/ttyTB_BASE` and `/dev/ttyTB_ARM` resolve correctly after **three** consecutive reboots
      with the cables in different physical ports
- [ ] Unplugging either USB cable mid-motion brakes the wheels within 100 ms
- [ ] Encoder counts increase monotonically when wheels are turned by hand, in the expected sign
- [ ] IMU yaw tracks chassis rotation with the expected sign, and does not drift more than a
      few degrees per minute at rest
- [ ] Round-trip `seq` echo latency is < 5 ms at the 99th percentile over a 10-minute run
- [ ] `CLAMP` with a deliberately stale timestamp is **rejected** by ESP-ARM
