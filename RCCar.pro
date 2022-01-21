QT -= gui
QT += network serialport

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp \
        sdvp_qtcommon/ext/pi-bno055/i2c_bno055.c \
        sdvp_qtcommon/bno055orientationupdater.cpp \
        sdvp_qtcommon/imuorientationupdater.cpp \
        sdvp_qtcommon/objectstate.cpp \
        sdvp_qtcommon/carstate.cpp \
        sdvp_qtcommon/ext/vesc/vescpacket.cpp \
        sdvp_qtcommon/legacy/locpoint.cpp \
        sdvp_qtcommon/legacy/packet.cpp \
        sdvp_qtcommon/legacy/tcpserversimple.cpp \
        sdvp_qtcommon/simplewatchdog.cpp \
        sdvp_qtcommon/vbytearray.cpp \
        sdvp_qtcommon/legacy/packetinterfacetcpserver.cpp \
        sdvp_qtcommon/movementcontroller.cpp \
        sdvp_qtcommon/motorcontroller.cpp \
        sdvp_qtcommon/servocontroller.cpp \
        sdvp_qtcommon/vescmotorcontroller.cpp \
        sdvp_qtcommon/vehiclestate.cpp \
        sdvp_qtcommon/pospoint.cpp \
        sdvp_qtcommon/carmovementcontroller.cpp \
        sdvp_qtcommon/gnss/ublox.cpp \
        sdvp_qtcommon/gnss/rtcm3_simple.cpp \
        sdvp_qtcommon/gnss/ubloxrover.cpp \
        sdvp_qtcommon/ext/Fusion/FusionBias.c \
        sdvp_qtcommon/ext/Fusion/FusionAhrs.c \
        sdvp_qtcommon/waypointfollower.cpp \
        sdvp_qtcommon/depthaicamera.cpp \
        sdvp_qtcommon/jsonstreamparsertcp.cpp \
        carpositionfuser.cpp

HEADERS += \
        sdvp_qtcommon/coordinatetransforms.h \
        sdvp_qtcommon/ext/pi-bno055/getbno055.h \
        sdvp_qtcommon/bno055orientationupdater.h \
        sdvp_qtcommon/imuorientationupdater.h \
        sdvp_qtcommon/objectstate.h \
        sdvp_qtcommon/carstate.h \
        sdvp_qtcommon/ext/vesc/vescpacket.h \
        sdvp_qtcommon/legacy/datatypes.h \
        sdvp_qtcommon/legacy/locpoint.h \
        sdvp_qtcommon/legacy/packet.h \
        sdvp_qtcommon/legacy/tcpserversimple.h \
        sdvp_qtcommon/simplewatchdog.h \
        sdvp_qtcommon/vbytearray.h \
        sdvp_qtcommon/legacy/packetinterfacetcpserver.h \
        sdvp_qtcommon/movementcontroller.h \
        sdvp_qtcommon/motorcontroller.h \
        sdvp_qtcommon/servocontroller.h \
        sdvp_qtcommon/vescmotorcontroller.h \
        sdvp_qtcommon/vehiclestate.h \
        sdvp_qtcommon/pospoint.h \
        sdvp_qtcommon/carmovementcontroller.h \
        sdvp_qtcommon/gnss/ublox.h \
        sdvp_qtcommon/gnss/rtcm3_simple.h \
        sdvp_qtcommon/gnss/ubloxrover.h \
        sdvp_qtcommon/ext/Fusion/FusionBias.h \
        sdvp_qtcommon/ext/Fusion/FusionAhrs.h \
        sdvp_qtcommon/ext/Fusion/FusionCalibration.h \
        sdvp_qtcommon/waypointfollower.h \
        sdvp_qtcommon/depthaicamera.h \
        sdvp_qtcommon/jsonstreamparsertcp.h \
        carpositionfuser.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
