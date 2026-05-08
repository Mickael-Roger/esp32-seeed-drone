idf_component_register(SRCS "icm20948_dmp.c" "icm20948.c" "icm20948_i2c.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC CONFIG_ICM_20948_USE_DMP=1)
