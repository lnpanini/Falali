# Exactly ONE sketch may be built — each defines setup()/loop().
#
#   sketch_bld.cpp  BLD-120A big-motor rig + Xbox gamepad   <-- current
#   sketch.cpp      L298N small bench rig + ToF auto-align
#
# Swap the two lines below to switch rigs.
set(srcs
        "main.c"
        "sketch_bld.cpp")
#       "sketch.cpp")

# VL53L0X is only needed by sketch.cpp's auto-align; harmless to keep listed.
set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "VL53L0X")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "." "../../src"
                    REQUIRES "${requires}")
