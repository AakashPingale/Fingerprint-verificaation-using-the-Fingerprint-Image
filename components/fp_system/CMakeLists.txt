# CMakeLists.txt for the fp_system component
#
# BEGINNER EXPLANATION:
# =====================
# An ESP-IDF "component" is a self-contained library. It has:
#   - Source files (.c)
#   - Header files (.h) in include/
#   - A CMakeLists.txt that registers it with the build system
#
# idf_component_register() tells ESP-IDF:
#   SRCS         - which .c files to compile
#   INCLUDE_DIRS - where to find .h files (for files inside this component)
#   REQUIRES     - which other components we need (ESP-IDF built-ins)

idf_component_register(
    SRCS
        "src/bt_serial.c"
        "src/cmd_parser.c"
        "src/fp_driver.c"
        "src/fp_storage.c"
        "src/fp_compare.c"
        "src/attendance.c"
        "src/fp_debug.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver          # UART driver, GPIO
        joltwallet__littlefs  # LittleFS (via component manager: joltwallet/littlefs)
        bt              # Bluetooth stack (bluedroid)
        nvs_flash       # Non-volatile storage (required by BT for bonding)
        esp_timer       # High-resolution timer (for packet timeouts)
        freertos        # FreeRTOS (tasks, queues, semaphores)
)
