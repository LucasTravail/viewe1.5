idf_component_register(SRCS "sdcard.c" ${SOURCES}
                    INCLUDE_DIRS . "include"
                    REQUIRES driver fatfs vfs lvgl)
