#!/usr/bin/env bash
# Bootstrap the Raspberry Pi 5 controller from the Mac, unattended.
#
#   ./tools/pi_bootstrap.sh
#
# Waits for the Pi to appear on the network, installs an SSH key, provisions the
# Python environment, copies pi/ across, and reports what hardware the Pi can see.
# Safe to re-run: every step is idempotent, and NOTHING here commands motor motion.
#
# The password is read from $TB_PASS or prompted for -- deliberately never written
# into this file, because this file is in git.
#
#   TB_PASS=... ./tools/pi_bootstrap.sh      # non-interactive
#   TB_USER=team10 TB_SUBNET=172.20.10 ./tools/pi_bootstrap.sh

set -uo pipefail

TB_USER="${TB_USER:-team10}"
TB_SUBNET="${TB_SUBNET:-172.20.10}"
TB_WAIT_MIN="${TB_WAIT_MIN:-15}"
KEY="$HOME/.ssh/id_ed25519"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

c()  { printf '\033[1;36m%s\033[0m\n' "$*"; }
ok() { printf '  \033[32mok\033[0m   %s\n' "$*"; }
no() { printf '  \033[31mFAIL\033[0m %s\n' "$*"; }

# --- 1. wait for the Pi ------------------------------------------------------
find_pi() {
  for i in $(seq 1 14); do ping -c1 -W 400 "$TB_SUBNET.$i" >/dev/null 2>&1 & done; wait
  for i in $(seq 1 14); do
    if nc -z -G 2 "$TB_SUBNET.$i" 22 2>/dev/null; then
      # A Pi answers with an OpenSSH banner; skip anything that isn't Linux.
      if nc -w 3 "$TB_SUBNET.$i" 22 </dev/null 2>&1 | head -1 | grep -qi "openssh.*debian\|openssh.*raspbian"; then
        echo "$TB_SUBNET.$i"; return 0
      fi
    fi
  done
  return 1
}

c "==> waiting for the Pi on $TB_SUBNET.0/28 (up to ${TB_WAIT_MIN} min)"
IP=""
deadline=$(( $(date +%s) + TB_WAIT_MIN * 60 ))
while [ -z "$IP" ]; do
  IP="$(find_pi || true)"
  [ -n "$IP" ] && break
  if [ "$(date +%s)" -ge "$deadline" ]; then
    no "Pi never appeared."
    echo "     Check: is the LED lit? Is it on its OWN power supply (not shared with the laptop)?"
    echo "     Is the Mac on the hotspot over WiFi (not USB tethering)?"
    exit 1
  fi
  printf '.'; sleep 10
done
echo; ok "Pi at $IP"

# --- 2. SSH key --------------------------------------------------------------
[ -f "$KEY" ] || ssh-keygen -t ed25519 -f "$KEY" -N "" -C "$(whoami)@mac-falali" >/dev/null

SSH="ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=10 $TB_USER@$IP"
if $SSH -o BatchMode=yes true 2>/dev/null; then
  ok "key auth already working"
else
  c "==> installing SSH key"
  if [ -z "${TB_PASS:-}" ]; then read -rsp "  password for $TB_USER@$IP: " TB_PASS; echo; fi
  /usr/bin/expect <<EXPECT >/dev/null 2>&1
set timeout 30
spawn ssh-copy-id -o StrictHostKeyChecking=accept-new -i $KEY.pub $TB_USER@$IP
expect {
  -re "(P|p)assword:" { send "$TB_PASS\r"; exp_continue }
  eof
}
EXPECT
  if $SSH -o BatchMode=yes true 2>/dev/null; then ok "key installed"; else no "key install failed - wrong password?"; exit 1; fi
fi

# --- 3. provision ------------------------------------------------------------
c "==> provisioning (apt, venv). This takes a few minutes on first run."
$SSH bash -s <<'REMOTE'
set -e
sudo apt-get update -qq
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-venv python3-pip git >/dev/null
# ModemManager probes fresh USB-serial devices with AT commands and can hold the
# port for seconds. It WILL interfere with the ESP link.
sudo systemctl disable --now ModemManager 2>/dev/null || true
sudo usermod -aG dialout "$USER"
mkdir -p ~/falali
[ -d ~/falali/.venv ] || python3 -m venv ~/falali/.venv
~/falali/.venv/bin/pip install -q --upgrade pip pyserial-asyncio-fast numpy
echo "provisioned"
REMOTE
[ $? -eq 0 ] && ok "packages + venv ready" || no "provisioning had errors (see above)"

# --- 4. copy the bridge ------------------------------------------------------
c "==> copying pi/ to the Pi"
rsync -az --delete -e "ssh -o StrictHostKeyChecking=accept-new" \
      "$REPO_ROOT/pi/" "$TB_USER@$IP:~/falali/pi/" && ok "pi/ synced" || no "rsync failed"

# --- 5. report what the Pi can see -------------------------------------------
c "==> hardware report (read-only, no motion commanded)"
$SSH bash -s <<'REMOTE'
echo "--- power / throttling (0x0 == never undervolted) ---"
vcgencmd get_throttled 2>/dev/null || echo "  vcgencmd unavailable"
echo "--- model / uptime ---"
tr -d '\0' < /proc/device-tree/model 2>/dev/null; echo
uptime
echo "--- USB devices ---"
lsusb 2>/dev/null | grep -viE "root hub" || echo "  (none)"
echo "--- serial ports (your ESP should be here) ---"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  NO SERIAL DEVICE -- is the ESP plugged in and powered?"
echo "--- USB serial identities (for udev rules) ---"
for d in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$d" ] || continue
  echo "  $d:"
  udevadm info -q property -n "$d" 2>/dev/null | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_VENDOR=" | sed 's/^/    /'
done
REMOTE

c "==> done"
echo "Next: ssh $TB_USER@$IP  then  cd ~/falali && .venv/bin/python -m pi.bridge --dry-run"
