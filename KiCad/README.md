# KiCad/ — custom PCB projects

Connector-level references for every board (extracted from these netlists):
[`docs/hardware/boards.md`](../docs/hardware/boards.md).

| Project | Size | Job |
|---|---|---|
| [`Wheel Drive PCB/`](Wheel%20Drive%20PCB/) | 130 × 105 mm | base ESP + 4× Driver PCB headers + all base sensors |
| [`Driver PCB/`](Driver%20PCB/) | 42.8 × 33.8 mm | 3.3 V → open-collector adapter for the BLD-120A, one per motor (need 4) |
| [`Arm Subsystem PCB/`](Arm%20Subsystem%20PCB/) | 110 × 110 mm | arm ESP + 2 BTS7960 H-bridges + servo power + 8 limit-switch headers |
| [`PCA9548A Breakout/`](PCA9548A%20Breakout/) | — | 8-ch I²C mux — historical; final firmware uses XSHUT re-addressing instead |

## Board renders

Every board carries schematic / PCB-layout / 3D-view images next to its sources
(the Driver PCB keeps its design-history set under `preview/`). Thumbnails below
are embedded — click any image for the full-resolution file:

| Board | Schematic | PCB layout | 3D view |
|---|---|---|---|
| Wheel Drive PCB | [![Wheel Drive schematic](Wheel%20Drive%20PCB/Wheel%20Drive%20Schematic%20image.png)](Wheel%20Drive%20PCB/Wheel%20Drive%20Schematic%20image.png) | [![Wheel Drive PCB layout](Wheel%20Drive%20PCB/Wheel%20Drive%20PCB%20image.png)](Wheel%20Drive%20PCB/Wheel%20Drive%20PCB%20image.png) | [![Wheel Drive 3D view](Wheel%20Drive%20PCB/Wheel%20Drive%203D%20View%20image.png)](Wheel%20Drive%20PCB/Wheel%20Drive%203D%20View%20image.png) |
| Driver PCB ×4 | — | [![Driver PCB final](Driver%20PCB/preview/board_final.png)](Driver%20PCB/preview/board_final.png) | [![Driver PCB 0.4 mm](Driver%20PCB/preview/board_0.4mm.png)](Driver%20PCB/preview/board_0.4mm.png) |
| Arm Subsystem PCB | [![Arm Subsystem schematic](Arm%20Subsystem%20PCB/Arm%20Subsystem%20Schematic%20Image.png)](Arm%20Subsystem%20PCB/Arm%20Subsystem%20Schematic%20Image.png) | [![Arm Subsystem PCB layout](Arm%20Subsystem%20PCB/Arm%20Subsystem%20PCB%20Image.png)](Arm%20Subsystem%20PCB/Arm%20Subsystem%20PCB%20Image.png) | [![Arm Subsystem 3D view](Arm%20Subsystem%20PCB/Arm%20Subsystem%203D%20View.png)](Arm%20Subsystem%20PCB/Arm%20Subsystem%203D%20View.png) |
| PCA9548A breakout | [![PCA9548A schematic](PCA9548A%20Breakout/PCA9548A%20Breakout%20Schematic.png)](PCA9548A%20Breakout/PCA9548A%20Breakout%20Schematic.png) | [![PCA9548A PCB layout](PCA9548A%20Breakout/PCA9548A%20Breakout%20PCB.png)](PCA9548A%20Breakout/PCA9548A%20Breakout%20PCB.png) | [![PCA9548A 3D view](PCA9548A%20Breakout/PCA9548A%20Breakout%203D%20View.png)](PCA9548A%20Breakout/PCA9548A%20Breakout%203D%20View.png) |

More Driver PCB renders — design history (placement, routing progress, copper
layers; click to open):

| | | |
|---|---|---|
| [![Placement](Driver%20PCB/preview/board_placed.png)](Driver%20PCB/preview/board_placed.png) | [![Routing progress](Driver%20PCB/preview/board_routed.png)](Driver%20PCB/preview/board_routed.png) | [![Top copper](Driver%20PCB/preview/board_top.png)](Driver%20PCB/preview/board_top.png) |
| [![Single bottom](Driver%20PCB/preview/board_single_bottom.png)](Driver%20PCB/preview/board_single_bottom.png) | [![Routed copper (SVG)](Driver%20PCB/preview/routed_copper.svg)](Driver%20PCB/preview/routed_copper.svg) | [![Bottom copper (SVG)](Driver%20PCB/preview/single_Bcu.svg)](Driver%20PCB/preview/single_Bcu.svg) |

Connector-level pin tables live in
[`docs/hardware/boards.md`](../docs/hardware/boards.md); these renders are the
visual companion to that reference.

Each project folder carries the KiCad sources (`.kicad_pro/.kicad_sch/.kicad_pcb`),
the extracted netlist (`.net`), schematic/PCB/3D renders as PNG, and JLCPCB fab
outputs (`fab-output/`; the Arm board's are zipped). Board designs: Kai Xiang &
Heng Li (Arm Subsystem, PCA9548A breakout), Bryan Lim (Wheel Drive, Driver).
