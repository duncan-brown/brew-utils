set(srcs "main.c"
    "st7789.c"
    "fontx.c"
    "arrow_test.c"
    "tlc5940.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
