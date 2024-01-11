cmake_minimum_required(VERSION 3.5)

project(RCCar LANGUAGES CXX C)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_STANDARD 23)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt5 COMPONENTS Core Network SerialPort REQUIRED)
find_package(MAVSDK REQUIRED)

add_executable(RCCar
    WayWise/autopilot/purepursuitwaypointfollower.cpp
    WayWise/autopilot/purepursuitwaypointfollower.h
    WayWise/autopilot/waypointfollower.h
    WayWise/autopilot/emergencybrake.cpp
    WayWise/communication/jsonstreamparsertcp.cpp
    WayWise/communication/vehicleserver.h
    WayWise/communication/mavsdkvehicleserver.cpp
    WayWise/communication/vehicleconnections/vehicleconnection.cpp
    WayWise/communication/mavlinkparameterserver.cpp
    WayWise/communication/parameterserver.cpp
    WayWise/core/pospoint.cpp
    WayWise/core/simplewatchdog.cpp
    WayWise/core/vbytearray.cpp
    WayWise/external/pi-bno055/i2c_bno055.c
    WayWise/external/vesc/vescpacket.cpp
    WayWise/sensors/camera/depthaicamera.cpp
    WayWise/sensors/camera/gimbal.h
    WayWise/sensors/fusion/sdvpvehiclepositionfuser.cpp
    WayWise/sensors/gnss/rtcm3_simple.cpp
    WayWise/sensors/gnss/rtcmclient.cpp
    WayWise/sensors/gnss/ublox.cpp
    WayWise/sensors/gnss/ubloxrover.cpp
    WayWise/sensors/imu/bno055orientationupdater.cpp
    WayWise/sensors/imu/imuorientationupdater.cpp
    WayWise/vehicles/carstate.cpp
    WayWise/vehicles/vehiclelighting.cpp
    WayWise/vehicles/controller/carmovementcontroller.cpp
    WayWise/vehicles/controller/motorcontroller.h
    WayWise/vehicles/controller/movementcontroller.cpp
    WayWise/vehicles/controller/servocontroller.cpp
    WayWise/vehicles/controller/vescmotorcontroller.cpp
    WayWise/vehicles/objectstate.cpp
    WayWise/vehicles/vehiclestate.cpp
    WayWise/logger/logger.cpp
    main.cpp
)

target_include_directories(RCCar PRIVATE WayWise/)

target_link_libraries(RCCar
    PRIVATE Qt5::Network
    PRIVATE Qt5::SerialPort
    PRIVATE MAVSDK::mavsdk
    PRIVATE gpiodcxx
)
