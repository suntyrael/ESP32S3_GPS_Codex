idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES gnss sensor input ui gpx pgear rtc log power
)
