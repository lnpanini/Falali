# KiCad/ — custom PCB projects

Connector-level references for every board (extracted from these netlists):
[`docs/hardware/boards.md`](../docs/hardware/boards.md).

| Project | Size | Job |
|---|---|---|
| [`Wheel Drive PCB/`](Wheel%20Drive%20PCB/) | 130 × 105 mm | base ESP + 4× Driver PCB headers + all base sensors |
| [`Driver PCB/`](Driver%20PCB/) | 42.8 × 33.8 mm | 3.3 V → open-collector adapter for the BLD-120A, one per motor (need 4) |
| [`Arm Subsystem PCB/`](Arm%20Subsystem%20PCB/) | 110 × 110 mm | arm ESP + 2 BTS7960 H-bridges + servo power + 8 limit-switch headers |
| [`PCA9548A Breakout/`](PCA9548A%20Breakout/) | — | 8-ch I²C mux — historical; final firmware uses XSHUT re-addressing instead |

Each project folder carries the KiCad sources (`.kicad_pro/.kicad_sch/.kicad_pcb`),
the extracted netlist (`.net`), schematic/PCB/3D renders as PNG, and JLCPCB fab
outputs (`fab-output/`; the Arm board's are zipped). Board designs: Kai Xiang &
Heng Li (Arm Subsystem, PCA9548A breakout), Bryan Lim (Wheel Drive, Driver).
