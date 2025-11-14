#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QDateTime>
#include <QFile>
#include <signal.h>
#include <QProcess>
#include "WayWise/core/simplewatchdog.h"
#include "WayWise/vehicles/carstate.h"
#include "WayWise/vehicles/controller/carmovementcontroller.h"
#include "WayWise/sensors/imu/bno055orientationupdater.h"
#include "WayWise/sensors/gnss/ubloxrover.h"
#include "WayWise/autopilot/waypointfollower.h"
#include "WayWise/autopilot/purepursuitwaypointfollower.h"
#include "WayWise/autopilot/emergencybrake.h"
#include "WayWise/vehicles/controller/vescmotorcontroller.h"
#include "WayWise/sensors/camera/depthaicamera.h"
#include "WayWise/sensors/fusion/sdvpvehiclepositionfuser.h"
#include "WayWise/sensors/gnss/rtcmclient.h"
#include "WayWise/communication/mavsdkvehicleserver.h"
#include "WayWise/communication/parameterserver.h"
#include "WayWise/logger/logger.h"
#include "WayWise/vehicles/vehiclelighting.h"
#include "WayWise/autopilot/followpoint.h"

static void terminationSignalHandler(int signal) {
    qDebug() << "Shutting down";
    if (signal==SIGINT || signal==SIGTERM || signal==SIGQUIT || signal==SIGHUP)
        qApp->quit();
}

int main(int argc, char *argv[])
{
    Logger::initVehicle();

    QCoreApplication a(argc, argv);
    const int mUpdateVehicleStatePeriod_ms = 25;
    QTimer mUpdateVehicleStateTimer;

    QSharedPointer<CarState> mCarState(new CarState);
    MavsdkVehicleServer mavsdkVehicleServer(mCarState/*, QHostAddress("192.168.111.255")*/); // <---! Add your IP of the Control Tower

    // --- Lower-level control setup ---
    QSharedPointer<CarMovementController> mCarMovementController(new CarMovementController(mCarState));
    // NOTE: HEADSTART rc car (values read from sdvp pcb)
    mCarMovementController->setSpeedToRPMFactor(2997.3);
    //mCarState->setAxisDistance(0.36);
    mCarState->setMaxSteeringAngle(atan(mCarState->getAxisDistance() / 0.67));

    // setup and connect VESC, simulate movements if unable to connect
    QSharedPointer<VESCMotorController> mVESCMotorController(new VESCMotorController());
    foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts()) {
        if (portInfo.description().toLower().replace("-", "").contains("chibios")) { // assumption: Serial device with ChibiOS in description is VESC
            mVESCMotorController->connectSerial(portInfo);
            qDebug() << "VESCMotorController connected to:" << portInfo.systemLocation();
        }
    }
    if (mVESCMotorController->isSerialConnected()) {
        mCarMovementController->setMotorController(mVESCMotorController);

        // VESC is a special case that can also control the servo
        const auto servoController = mVESCMotorController->getServoController();
        servoController->setInvertOutput(true);
        // NOTE: HEADSTART rc car (values read from sdvp pcb)
        servoController->setServoRange(0.50);
        servoController->setServoCenter(0.5);
        mCarMovementController->setServoController(servoController);
    } else {
        QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [&](){
            mCarMovementController->simulationStep(mUpdateVehicleStatePeriod_ms);
        });
        mUpdateVehicleStateTimer.start(mUpdateVehicleStatePeriod_ms);
    }

    // --- Positioning setup ---
    // Position Fuser
    SDVPVehiclePositionFuser positionFuser;

    // GNSS (with fused IMU when using u-blox F9R)
    QSharedPointer<GNSSReceiver> mGNSSReceiver;
    foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts()) {
        if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
            QSharedPointer<UbloxRover> mUbloxRover(new UbloxRover(mCarState));
            if (mUbloxRover->connectSerial(portInfo)) {
                qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();

                mUbloxRover->setChipOrientationOffset(0.0, 0.0, 0.0);
                QObject::connect(mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw, &positionFuser, &SDVPVehiclePositionFuser::correctPositionAndYawGNSS);

                mUbloxRover->setReceiverVariant(RECEIVER_VARIANT::UBLX_ZED_F9P); // or UBLX_ZED_F9P
                mavsdkVehicleServer.setUbloxRover(mUbloxRover);

                // -- NTRIP/TCP client setup for feeding RTCM data into GNSS receiver
                RtcmClient rtcmClient;
                QObject::connect(mUbloxRover.get(), &UbloxRover::gotNmeaGga, &rtcmClient, &RtcmClient::forwardNmeaGgaToServer);
                QObject::connect(&rtcmClient, &RtcmClient::rtcmData, mUbloxRover.get(), &UbloxRover::writeRtcmToUblox);
                QObject::connect(&rtcmClient, &RtcmClient::baseStationPosition, mUbloxRover.get(), &UbloxRover::setEnuRef);
                if (rtcmClient.connectWithInfoFromFile("./rtcmServerInfo.txt"))
                    qDebug() << "RtcmClient: connected to" << QString(rtcmClient.getCurrentHost()+ ":" + QString::number(rtcmClient.getCurrentPort()));
                else
                    qDebug() << "RtcmClient: not connected";

                mGNSSReceiver = mUbloxRover;
            }
        }
    }

    if (!mGNSSReceiver) {
        qDebug() << "No GNSS receiver connected. Simulating GNSS data.";
        mGNSSReceiver.reset(new GNSSReceiver(mCarState));
        mGNSSReceiver->setReceiverVariant(RECEIVER_VARIANT::WAYWISE_SIMULATED);
        mGNSSReceiver->setReceiverState(RECEIVER_STATE::READY);

        QObject::connect(mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, [&](QSharedPointer<VehicleState> vehicleState, double distanceDriven){
            Q_UNUSED(distanceDriven)
            PosPoint currentPosition = vehicleState->getPosition(PosType::odom);
            currentPosition.setType(PosType::GNSS);
            vehicleState->setPosition(currentPosition);
            currentPosition.setType(PosType::fused);
            vehicleState->setPosition(currentPosition);
        });
    }

    // TODO: re-add feature for MAVLink-based communication
    // -- In case RControlStation sends RTCM data, RtcmClient is disabled (RTCM from RControlStation has priority)
//    QObject::connect(&mPacketIFServer, &PacketInterfaceTCPServer::rtcmData, [&](){
//        qDebug() << "PacketInterfaceTCPServer: got RTCM data, disabling on-vehicle RTCM client.";
//        rtcmClient.disconnect();
//        QObject::disconnect(&rtcmClient, &RtcmClient::rtcmData, mUbloxRover.get(), &UbloxRover::writeRtcmToUblox);
//        QObject::disconnect(&rtcmClient, &RtcmClient::baseStationPosition, mUbloxRover.get(), &UbloxRover::setEnuRef);
//        QObject::disconnect(&mPacketIFServer, &PacketInterfaceTCPServer::rtcmData, nullptr, nullptr); // run this slot only once
//    });

    // IMU
    bool useVESCIMU = true;
    QSharedPointer<IMUOrientationUpdater> mIMUOrientationUpdater;
    if (useVESCIMU) {
        if (mVESCMotorController->isSerialConnected()) {
            mIMUOrientationUpdater = mVESCMotorController->getIMUOrientationUpdater(mCarState);
        } else {
            qDebug() << "VESCMotorController not connected. Simulating IMU data.";
            QObject::connect(mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, [&](QSharedPointer<VehicleState> vehicleState, double distanceDriven){
                Q_UNUSED(distanceDriven)
                PosPoint currentPosition = vehicleState->getPosition(PosType::odom);
                currentPosition.setType(PosType::IMU);
                vehicleState->setPosition(currentPosition);
                positionFuser.correctPositionAndYawIMU(vehicleState);
            });
        }
    } else {
        mIMUOrientationUpdater.reset(new BNO055OrientationUpdater(mCarState, "/dev/i2c-1"));
    }
    QObject::connect(mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation, &positionFuser, &SDVPVehiclePositionFuser::correctPositionAndYawIMU);

    // Odometry
    QObject::connect(mCarMovementController.get(), &CarMovementController::updatedOdomPositionAndYaw, &positionFuser, &SDVPVehiclePositionFuser::correctPositionAndYawOdom);

    // TODO: input to u-blox disabled for now, seems to cause problems (lost fusion mode on F9R) and needs testing/debugging
//    QObject::connect(mVESCMotorController.get(), &VESCMotorController::gotStatusValues, [&](double rpm, int tachometer, int tachometer_abs){
//       Q_UNUSED(rpm)
//       uint32_t ticks = tachometer_abs;
//       uint32_t wheelTickMax = 8388607;
//       ticks &=  wheelTickMax; // Bits 31..23 are set to zero

//       static int previousTachometer = 0;
//       bool direction = ((tachometer - previousTachometer) > previousTachometer);
//       ticks |= direction << 23;

//       mUbloxRover->writeOdoToUblox(SINGLE_TICK,ticks);

//       previousTachometer = tachometer;
//    });

    // --- Autopilot ---
    QSharedPointer<PurepursuitWaypointFollower> mWaypointFollower(new PurepursuitWaypointFollower(mCarMovementController));
    mWaypointFollower->setPurePursuitRadius(1.0);
    mWaypointFollower->setRepeatRoute(false);
    mWaypointFollower->setAdaptivePurePursuitRadiusActive(true);

    // --- Follow Point ---
    QSharedPointer<FollowPoint> mFollowPoint(new FollowPoint(mCarMovementController));

    // DepthAI Camera
    DepthAiCamera mDepthAiCamera;
    QObject::connect(&mDepthAiCamera, &DepthAiCamera::closestObject, mFollowPoint.get(), &FollowPoint::updatePointToFollowInVehicleFrame);

    // Emergency brake
    EmergencyBrake mEmergencyBrake;
    QObject::connect(&mDepthAiCamera, &DepthAiCamera::closestObject, &mEmergencyBrake, &EmergencyBrake::brakeForDetectedCameraObject);
    QObject::connect(mWaypointFollower.get(), &WaypointFollower::deactivateEmergencyBrake, &mEmergencyBrake, &EmergencyBrake::deactivateEmergencyBrake);
    QObject::connect(mWaypointFollower.get(), &WaypointFollower::activateEmergencyBrake, &mEmergencyBrake, &EmergencyBrake::activateEmergencyBrake);
    QObject::connect(mFollowPoint.get(), &FollowPoint::deactivateEmergencyBrake, &mEmergencyBrake, &EmergencyBrake::deactivateEmergencyBrake);
    QObject::connect(mFollowPoint.get(), &FollowPoint::activateEmergencyBrake, &mEmergencyBrake, &EmergencyBrake::activateEmergencyBrake);

    QObject::connect(&mEmergencyBrake, &EmergencyBrake::emergencyBrake, mWaypointFollower.get(), &WaypointFollower::stop);

    // Vehicle lighting
    QSharedPointer<VehicleLighting> mVehicleLighting(new VehicleLighting(mCarState));

    // Setup MAVLINK communication towards ControlTower
    mavsdkVehicleServer.setMovementController(mCarMovementController);
    mavsdkVehicleServer.setWaypointFollower(mWaypointFollower);
    mavsdkVehicleServer.setFollowPoint(mFollowPoint);
    // TODO: motor controller status not supported in ControlTower

    // Advertise parameters
    mCarState->provideParametersToParameterServer();
    mavsdkVehicleServer.provideParametersToParameterServer();
    mWaypointFollower->provideParametersToParameterServer();
    mFollowPoint->provideParametersToParameterServer();

    // Watchdog that warns when EventLoop is slowed down
    SimpleWatchdog watchdog;

    // Perform safe shutdown
    signal(SIGINT, terminationSignalHandler);
    QObject::connect(&a, &QCoreApplication::aboutToQuit, [&](){
        mGNSSReceiver->aboutToShutdown();
        ParameterServer::getInstance()->saveParametersToXmlFile("vehicle_parameters.xml");
    });
    QObject::connect(&mavsdkVehicleServer, &MavsdkVehicleServer::shutdownOrRebootOnboardComputer, [&](bool isShutdown){
        qApp->quit();
        if (isShutdown) {
           qDebug() << "\nSystem shutdown...";
           QProcess::startDetached("sudo", QStringList() << "shutdown" << "-P" << "now");
        }else {
           qDebug() << "\nSystem reboot...";
           QProcess::startDetached("sudo", QStringList() << "shutdown" << "-r" << "now");
        }
    });

    qDebug() << "\n" // by hjw
             << "                    .------.\n"
             << "                    :|||\"\"\"`.`.\n"
             << "                    :|||     7.`.\n"
             << " .===+===+===+===+===||`----L7'-`7`---.._\n"
             << " []                  || ==       |       \"\"\"-.\n"
             << " []...._____.........||........../ _____ ____|\n"
             << "c\\____/,---.\\_       ||_________/ /,---.\\_  _/\n"
             << "  /_,-/ ,-. \\ `._____|__________||/ ,-. \\ \\_[\n"
             << "     /\\ `-' /                    /\\ `-' /\n"
             << "       `---'                       `---'\n";

    return a.exec();
}
