# tools/ — bench & production tooling

| Tool | What it is |
|---|---|
| `board_test.py` | production test loop for manufactured Wheel Drive PCBs: runs `env:pcb_rl` against board after board, records results to CSV |
| `board_test_log.csv` | logged results from real board-test sessions |
| `golden_signature.json` | expected GPIO states extracted from the netlist — what a healthy board must match |
| `magtune.py` | live AS5600 magnet-airgap tuner: polls the bench firmware and renders AGC as a bar to position the magnet by eye |
| `find_pi.sh` | sweep the local network for the Raspberry Pi 5 |
| `pi_bootstrap.sh` | unattended Pi 5 setup from a laptop (see [`pi/`](../pi/)) |
