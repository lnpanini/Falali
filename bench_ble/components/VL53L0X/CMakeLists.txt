# Pololu VL53L0X Arduino library, vendored as an ESP-IDF component so the
# Bluepad32/ESP-IDF build can use it (plain lib_deps doesn't apply here).
idf_component_register(SRCS "VL53L0X.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES arduino)
