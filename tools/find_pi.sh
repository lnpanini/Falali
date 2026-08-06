#!/usr/bin/env bash
# Is the Pi on the network yet?  ->  ./tools/find_pi.sh
#
# Auto-detects whichever network the Mac is on, sweeps it, reports the Pi.
# Read-only: pings and a port check, nothing else. Takes ~5 seconds.
#
#   ./tools/find_pi.sh          # scan (hosts .1-30, covers a phone hotspot /28)
#   ./tools/find_pi.sh -f       # full /24 sweep, for a real router
#   ./tools/find_pi.sh -w       # keep scanning until it appears, then beep

set -uo pipefail
TB_USER="${TB_USER:-team10}"
HI=30; WATCH=0
for a in "$@"; do
  case "$a" in
    -f) HI=254 ;;
    -w) WATCH=1 ;;
  esac
done

find_iface_ip() {
  for i in $(ifconfig -l 2>/dev/null); do
    ip=$(ipconfig getifaddr "$i" 2>/dev/null) || continue
    [ -n "$ip" ] && { echo "$i $ip"; return 0; }
  done
  return 1
}

scan() {
  read -r IFACE MYIP <<<"$(find_iface_ip)" || { echo "Mac has no network connection."; return 1; }
  SUB="${MYIP%.*}"
  echo "Mac: $MYIP on $IFACE   scanning $SUB.1-$HI"

  for i in $(seq 1 $HI); do ping -c1 -W 300 "$SUB.$i" >/dev/null 2>&1 & done; wait

  echo "--- devices seen ---"
  arp -a -n 2>/dev/null | grep "$SUB\." | grep -v incomplete | grep -v 'ff:ff:ff' | sed 's/^/  /'

  # Port checks in parallel -- serial nc across a whole /24 takes minutes.
  echo "--- SSH (port 22) ---"
  TMP=$(mktemp)
  for i in $(seq 1 $HI); do
    ( nc -z -G 1 "$SUB.$i" 22 2>/dev/null && echo "$SUB.$i" >> "$TMP" ) &
  done; wait

  if [ ! -s "$TMP" ]; then
    rm -f "$TMP"; echo "  none"; echo; echo ">>> PI NOT FOUND"; return 1
  fi
  sed 's/^/  OPEN  /' "$TMP"
  FOUND=$(head -1 "$TMP"); rm -f "$TMP"

  BANNER=$(nc -w 3 "$FOUND" 22 </dev/null 2>&1 | head -1)
  echo "--- banner ---"; echo "  $BANNER"
  case "$BANNER" in
    *Debian*|*Raspbian*) echo; echo ">>> PI FOUND AT $FOUND" ;;
    *)                   echo; echo ">>> host at $FOUND, but may not be the Pi" ;;
  esac

  if ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=6 \
        "$TB_USER@$FOUND" true 2>/dev/null; then
    echo "    key auth WORKS  ->  ssh $TB_USER@$FOUND"
  else
    echo "    key not installed  ->  ./tools/pi_bootstrap.sh"
  fi
  return 0
}

if [ "$WATCH" -eq 1 ]; then
  echo "watching until the Pi appears (Ctrl-C to stop)..."
  until scan; do sleep 15; echo; done
  printf '\a'
else
  scan
fi
