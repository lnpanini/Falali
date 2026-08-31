# tools/ — bench & production tooling

| Tool | What it is |
|---|---|
| `board_test.py` | production test loop for manufactured Wheel Drive PCBs: runs `env:pcb_rl` against board after board, records results to CSV |
| `board_test_log.csv` | logged results from real board-test sessions |
| `golden_signature.json` | expected GPIO states extracted from the netlist — what a healthy board must match |

`magtune.py` (AS5600 magnet tuner), `find_pi.sh` and `pi_bootstrap.sh` were
removed 2026-08-31 with the encoders and the Raspberry Pi 5 plan they served.
