# Exactly ONE sketch may be built — each defines setup()/loop().
#
# sketch_pcb.cpp is the only sketch. It targets the FABRICATED Wheel Drive PCB
# and reads include/pins.h.
#
# Two earlier sketches were removed 2026-08-31: sketch_bld.cpp (BLD-120A
# breadboard rig) and sketch.cpp (L298N downscale prototype). Both carried pin
# tables measured on breadboards, where the FR and RL corners have SV and BRK
# swapped relative to the fabricated board -- building either against this board
# would have driven speed PWM into two brake lines.
set(srcs
        "main.c"
        "sketch_pcb.cpp"
        # the REAL alignment stack, reused from the production firmware
        "../../lib/domain/CornerEdgeDetector.cpp"
        "../../lib/domain/DeadReckonOdometry.cpp"
        "../../lib/domain/DockingStateMachine.cpp"
        "../../lib/domain/SafetyMonitor.cpp")

# VL53L0X is only needed by sketch.cpp's auto-align; harmless to keep listed.
set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "VL53L0X")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "." "../../src" "../../include"
                                 "../../lib/ports" "../../lib/domain" "../../lib/hal_esp32"
                    REQUIRES "${requires}")
