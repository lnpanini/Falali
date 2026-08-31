# lib/ — the hexagonal core

| Folder | What it is |
|---|---|
| [`ports/`](ports/) | pure C++ interfaces + shared types — the only thing domain code depends on (no Arduino here) |
| [`domain/`](domain/) | the hand-written logic under test: `CornerEdgeDetector`, `DeadReckonOdometry`, `DockingStateMachine`, `SafetyMonitor`, `StallDetector`, `LinkWatchdog`, `MotorCalAnalysis` |
| [`drive/`](drive/) | `MecanumDrive` — mixes body-frame vx/vy/ω into four wheel speeds |
| [`fakes/`](fakes/) | desktop test doubles for every port — this is why `pio test -e native` needs no hardware |
| [`hal_esp32/`](hal_esp32/) | Arduino adapters over popular libraries: VL53L0X array (XSHUT re-addressing), BLD-120A driver, BTS7960 clamp, ADS1115 current sense, BNO085 IMU, telemetry |

Dependency rule: **domain → ports** only; **adapters → ports**; **main → everything**.
