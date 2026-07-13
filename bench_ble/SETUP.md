# bench_ble — Bluepad32 gamepad build

Separate PlatformIO project (ESP-IDF + Arduino-as-component) for driving the bench rig
with an Xbox BLE controller. It exists apart from the root Arduino project because
Bluepad32 replaces the Bluetooth stack with BTstack and ships its own framework build.

Generated from: https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template
(pioarduino platform, ESP-IDF v5.4.2). Bluepad32 for Arduino v4.2.0.

## Vendored components (git-ignored)

`components/` (arduino, bluepad32, bluepad32_arduino, btstack, cmd_*) and all build
artifacts are git-ignored to keep the repo lean. Re-fetch them on a fresh checkout with:

    git clone --recursive https://github.com/ricardoquesada/esp-idf-arduino-bluepad32-template _tpl
    rsync -a _tpl/components/ components/ && rm -rf _tpl

(The component URLs are recorded in `.gitmodules`.)

## Build / flash / monitor

    pio run  -d bench_ble -e esp32-s3-devkitc-1            # build
    pio run  -d bench_ble -e esp32-s3-devkitc-1 -t upload  # flash (native USB / usbmodem*)
    python teleop.py                                        # monitor (glob finds usbmodem*)

## Pairing an Xbox controller

Xbox Wireless **model 1914** (Series X/S), firmware >= v5.15, BLE-only. Turn it on, then
hold the **Pair** button ~3 s until it flashes fast. Bluepad32 auto-connects; you should see
`CALLBACK: Controller is connected` + `Controller model: Xbox Wireless Controller`.

## Task 4 note: Console vs Serial

The stock sketch prints via Bluepad32's `Console` (default `CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=y`),
which is incompatible with Arduino `Serial`. Our shared core (`../src/bench_core.h`) prints via
`Serial`, so when wiring it in (Task 4) set `CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=n` in
`sdkconfig.defaults`, and add `-I ${PROJECT_DIR}/../src` + the Pololu VL53L0X component.
