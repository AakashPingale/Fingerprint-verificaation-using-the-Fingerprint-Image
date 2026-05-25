idf_component_register(
    SRCS "FP_Module.c"
    INCLUDE_DIRS "."
    # Depend on our custom fp_system component and NVS (needed by BT)
    REQUIRES
        fp_system
        nvs_flash
)
