# test/ — host unit tests

One suite per domain module (Unity framework). They run on the laptop against
[`lib/fakes/`](../lib/fakes/) — no hardware needed:

```bash
pio test -e native
```

Covered: corner edge detection, dead-reckon odometry, docking state machine,
safety monitor, stall detector, link watchdog, mecanum mixing, motor-cal analysis.
