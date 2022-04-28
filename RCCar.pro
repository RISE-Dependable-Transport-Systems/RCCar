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

INCLUDEPATH += sdvp_qtcommon/

SOURCES += \
        main.cpp \
        sdvp_qtcommon/external/pi-bno055/i2c_bno055.c \
        sdvp_qtcommon/sensors/imu/bno055orientationupdater.cpp \
        sdvp_qtcommon/sensors/imu/imuorientationupdater.cpp \
        sdvp_qtcommon/vehicles/objectstate.cpp \
        sdvp_qtcommon/vehicles/carstate.cpp \
        sdvp_qtcommon/external/vesc/vescpacket.cpp \
        sdvp_qtcommon/legacy/locpoint.cpp \
        sdvp_qtcommon/legacy/packet.cpp \
        sdvp_qtcommon/legacy/tcpserversimple.cpp \
        sdvp_qtcommon/core/simplewatchdog.cpp \
        sdvp_qtcommon/core/vbytearray.cpp \
        sdvp_qtcommon/legacy/packetinterfacetcpserver.cpp \
        sdvp_qtcommon/vehicles/controller/movementcontroller.cpp \
        sdvp_qtcommon/vehicles/controller/motorcontroller.cpp \
        sdvp_qtcommon/vehicles/controller/servocontroller.cpp \
        sdvp_qtcommon/vehicles/controller/vescmotorcontroller.cpp \
        sdvp_qtcommon/vehicles/vehiclestate.cpp \
        sdvp_qtcommon/core/pospoint.cpp \
        sdvp_qtcommon/vehicles/controller/carmovementcontroller.cpp \
        sdvp_qtcommon/sensors/gnss/ublox.cpp \
        sdvp_qtcommon/sensors/gnss/rtcm3_simple.cpp \
        sdvp_qtcommon/sensors/gnss/ubloxrover.cpp \
        sdvp_qtcommon/sensors/gnss/rtcmclient.cpp \
        sdvp_qtcommon/autopilot/waypointfollower.cpp \
        sdvp_qtcommon/sensors/camera/depthaicamera.cpp \
        sdvp_qtcommon/communication/jsonstreamparsertcp.cpp \
        sdvp_qtcommon/sensors/fusion/sdvpvehiclepositionfuser.cpp

HEADERS += \
        sdvp_qtcommon/core/coordinatetransforms.h \
        sdvp_qtcommon/external/pi-bno055/getbno055.h \
        sdvp_qtcommon/sensors/imu/bno055orientationupdater.h \
        sdvp_qtcommon/sensors/imu/imuorientationupdater.h \
        sdvp_qtcommon/vehicles/objectstate.h \
        sdvp_qtcommon/vehicles/carstate.h \
        sdvp_qtcommon/external/vesc/vescpacket.h \
        sdvp_qtcommon/legacy/datatypes.h \
        sdvp_qtcommon/legacy/locpoint.h \
        sdvp_qtcommon/legacy/packet.h \
        sdvp_qtcommon/legacy/tcpserversimple.h \
        sdvp_qtcommon/core/simplewatchdog.h \
        sdvp_qtcommon/core/vbytearray.h \
        sdvp_qtcommon/legacy/packetinterfacetcpserver.h \
        sdvp_qtcommon/vehicles/controller/movementcontroller.h \
        sdvp_qtcommon/vehicles/controller/motorcontroller.h \
        sdvp_qtcommon/vehicles/controller/servocontroller.h \
        sdvp_qtcommon/vehicles/controller/vescmotorcontroller.h \
        sdvp_qtcommon/vehicles/vehiclestate.h \
        sdvp_qtcommon/core/pospoint.h \
        sdvp_qtcommon/vehicles/controller/carmovementcontroller.h \
        sdvp_qtcommon/sensors/gnss/ublox.h \
        sdvp_qtcommon/sensors/gnss/rtcm3_simple.h \
        sdvp_qtcommon/sensors/gnss/ubloxrover.h \
        sdvp_qtcommon/sensors/gnss/rtcmclient.h \
        sdvp_qtcommon/autopilot/waypointfollower.h \
        sdvp_qtcommon/sensors/camera/depthaicamera.h \
        sdvp_qtcommon/communication/jsonstreamparsertcp.h \
        sdvp_qtcommon/sensors/fusion/sdvpvehiclepositionfuser.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
