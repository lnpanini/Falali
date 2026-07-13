set(srcs
        "main.c"
        "sketch.cpp")

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "VL53L0X")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "." "../../src"
                    REQUIRES "${requires}")
