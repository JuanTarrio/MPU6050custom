
TARGET = mpu60580


INCLUDEPATH = /usr/src/linux-headers-4.9.35-v7+/include/
SOURCES += mpu6050_custom.c \
    read_event.cpp
HEADERS +=
OTHER_FILES += build.sh
