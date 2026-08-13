# Exactly ONE sketch may be built — each defines setup()/loop().
#
#   sketch_pcb.cpp  FABRICATED Wheel Drive PCB, pins.h map    <-- current
#   sketch_bld.cpp  BLD-120A BREADBOARD rig + Xbox gamepad
#   sketch.cpp      L298N small bench rig + ToF auto-align
#
# *** DO NOT BUILD sketch_bld.cpp FOR THE FABRICATED BOARD. ***
# Its pin table was measured on the breadboard, and on the PCB the FR and RL
# corners have SV and BRK swapped -- it would drive speed PWM into two brake
# lines. sketch_pcb.cpp reads include/pins.h instead.
#
# Swap the lines below to switch rigs.
set(srcs
        "main.c"
        "sketch_pcb.cpp"
        # the REAL alignment stack, reused from the production firmware
        "../../lib/domain/CornerEdgeDetector.cpp"
        "../../lib/domain/DeadReckonOdometry.cpp"
        "../../lib/domain/DockingStateMachine.cpp"
        "../../lib/domain/SafetyMonitor.cpp")
#       "sketch_bld.cpp")
#       "sketch.cpp")

# VL53L0X is only needed by sketch.cpp's auto-align; harmless to keep listed.
set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "VL53L0X")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "." "../../src" "../../include"
                                 "../../lib/ports" "../../lib/domain" "../../lib/hal_esp32"
                    REQUIRES "${requires}")
