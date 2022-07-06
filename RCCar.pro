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

INCLUDEPATH += WayWise/

SOURCES += \
        main.cpp \
        WayWise/external/pi-bno055/i2c_bno055.c \
        WayWise/sensors/imu/bno055orientationupdater.cpp \
        WayWise/sensors/imu/imuorientationupdater.cpp \
        WayWise/vehicles/objectstate.cpp \
        WayWise/vehicles/carstate.cpp \
        WayWise/external/vesc/vescpacket.cpp \
        WayWise/legacy/locpoint.cpp \
        WayWise/legacy/packet.cpp \
        WayWise/legacy/tcpserversimple.cpp \
        WayWise/core/simplewatchdog.cpp \
        WayWise/core/vbytearray.cpp \
        WayWise/legacy/packetinterfacetcpserver.cpp \
        WayWise/vehicles/controller/movementcontroller.cpp \
        WayWise/vehicles/controller/servocontroller.cpp \
        WayWise/vehicles/controller/vescmotorcontroller.cpp \
        WayWise/vehicles/vehiclestate.cpp \
        WayWise/core/pospoint.cpp \
        WayWise/vehicles/controller/carmovementcontroller.cpp \
        WayWise/sensors/gnss/ublox.cpp \
        WayWise/sensors/gnss/rtcm3_simple.cpp \
        WayWise/sensors/gnss/ubloxrover.cpp \
        WayWise/sensors/gnss/rtcmclient.cpp \
#        WayWise/communication/vehicleconnections/vehicleconnection.cpp \
        WayWise/autopilot/purepursuitwaypointfollower.cpp \
        WayWise/sensors/camera/depthaicamera.cpp \
        WayWise/communication/jsonstreamparsertcp.cpp \
        WayWise/sensors/fusion/sdvpvehiclepositionfuser.cpp

HEADERS += \
        WayWise/core/coordinatetransforms.h \
        WayWise/external/pi-bno055/getbno055.h \
        WayWise/sensors/imu/bno055orientationupdater.h \
        WayWise/sensors/imu/imuorientationupdater.h \
        WayWise/vehicles/objectstate.h \
        WayWise/vehicles/carstate.h \
        WayWise/external/vesc/vescpacket.h \
        WayWise/legacy/datatypes.h \
        WayWise/legacy/locpoint.h \
        WayWise/legacy/packet.h \
        WayWise/legacy/tcpserversimple.h \
        WayWise/core/simplewatchdog.h \
        WayWise/core/vbytearray.h \
        WayWise/legacy/packetinterfacetcpserver.h \
        WayWise/vehicles/controller/movementcontroller.h \
        WayWise/vehicles/controller/motorcontroller.h \
        WayWise/vehicles/controller/servocontroller.h \
        WayWise/vehicles/controller/vescmotorcontroller.h \
        WayWise/vehicles/vehiclestate.h \
        WayWise/core/pospoint.h \
        WayWise/vehicles/controller/carmovementcontroller.h \
        WayWise/sensors/gnss/ublox.h \
        WayWise/sensors/gnss/rtcm3_simple.h \
        WayWise/sensors/gnss/ubloxrover.h \
        WayWise/sensors/gnss/rtcmclient.h \
        WayWise/autopilot/waypointfollower.h \
        WayWise/sensors/camera/gimbal.h \
        WayWise/communication/vehicleconnections/vehicleconnection.h \
        WayWise/autopilot/purepursuitwaypointfollower.h \
        WayWise/sensors/camera/depthaicamera.h \
        WayWise/communication/jsonstreamparsertcp.h \
        WayWise/sensors/fusion/sdvpvehiclepositionfuser.h

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
