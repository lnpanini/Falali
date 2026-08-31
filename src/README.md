# src/ — firmware entry points & bench sketches

| File | What it is |
|---|---|
| `main.cpp` | the docking firmware — composition root: builds adapters, wires the hexagonal core, runs the loop |
| `bench_check.cpp` | interactive bench sketch (drive/align/status over serial) — pair with [`teleop.py`](../teleop.py) |
| `bench_pcb_rl.cpp` | production-test firmware for the Wheel Drive PCB (`env:pcb_rl`); driven by [`tools/board_test.py`](../tools/board_test.py) |
| `bench_s3_motor.cpp`, `bench_motor.cpp` | BLDC motor bring-up on the DevKit / on the Wheel Drive PCB |
| `i2c_scan.cpp`, `tof_scan.cpp` | bus utilities: scan the I²C bus, probe VL53L0X sensors |
| `pcb_identify.cpp` | identifies which PCB a DevKit is plugged into by probing its peripherals |
| `bench_align.h`, `bench_drive.h`, `bench_mix.h`, `bench_bld_drive.h` | focused bench exercises for one subsystem at a time |

PlatformIO environments for all of these live in [`platformio.ini`](../platformio.ini).
