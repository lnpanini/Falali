# src/ — firmware entry points & bench sketches

| File | What it is |
|---|---|
| `main.cpp` | the docking firmware — composition root: builds adapters, wires the hexagonal core, runs the loop |
| `bench_pcb_rl.cpp` | production-test firmware for the Wheel Drive PCB (`env:pcb_rl`); driven by [`tools/board_test.py`](../tools/board_test.py) |
| `i2c_scan.cpp`, `tof_scan.cpp` | bus utilities: scan the I²C bus, probe VL53L0X sensors |
| `pcb_identify.cpp` | identifies which PCB a DevKit is plugged into by probing its peripherals |
| `bench_mix.h` | mecanum body-frame → wheel mix, shared with the base firmware |

PlatformIO environments for all of these live in [`platformio.ini`](../platformio.ini).
