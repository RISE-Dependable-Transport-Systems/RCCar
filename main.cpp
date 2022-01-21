#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QDateTime>
#include "sdvp_qtcommon/simplewatchdog.h"
#include "sdvp_qtcommon/legacy/packetinterfacetcpserver.h"
#include "sdvp_qtcommon/carstate.h"
#include "sdvp_qtcommon/carmovementcontroller.h"
#include "sdvp_qtcommon/bno055orientationupdater.h"
#include "sdvp_qtcommon/gnss/ubloxrover.h"
#include "sdvp_qtcommon/waypointfollower.h"
#include "sdvp_qtcommon/vescmotorcontroller.h"
#include "sdvp_qtcommon/depthaicamera.h"
#include "carpositionfuser.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    bool isSimulation = false;

    // --- VehicleState and lower-level control setup ---
    QSharedPointer<CarState> mCarState(new CarState);
    QSharedPointer<CarMovementController> mCarMovementController(new CarMovementController(mCarState));
    // NOTE: HEADSTART rc car (values read from sdvp pcb)
    mCarMovementController->setSpeedToRPMFactor(2997.3);
    mCarState->setAxisDistance(0.36);
    mCarState->setMaxSteeringAngle(atan(mCarState->getAxisDistance() / 0.67));

    // setup and connect VESC
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
    } else
        isSimulation = true;

    // --- Positioning setup ---
    // Position Fuser
    // TODO: refactor inputs to CarPositionFuser (need to be moved to respective classes). This was quickly implemented for HEADSTART
    CarPositionFuser positionFuser;

    // IMU
    bool useVESCIMU = true;
    QSharedPointer<IMUOrientationUpdater> mIMUOrientationUpdater;
    if (useVESCIMU)
        mIMUOrientationUpdater = mVESCMotorController->getIMUOrientationUpdater(mCarState);
    else
        mIMUOrientationUpdater.reset(new BNO055OrientationUpdater(mCarState, "/dev/i2c-3"));
    // Take roll & pitch, yaw from VESC's IMU
    QObject::connect(mIMUOrientationUpdater.get(), &IMUOrientationUpdater::updatedIMUOrientation, &positionFuser, &CarPositionFuser::correctPositionAndYawIMU);

    // Odometry
    // TODO: no real PosType::odom update implemented right now. This is just for simulation
    const int mUpdateVehicleStatePeriod_ms = 25;
    QTimer mUpdateVehicleStateTimer;
    if (isSimulation) {
        QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [&](){
            mCarState->simulationStep(mUpdateVehicleStatePeriod_ms, PosType::fused);
        });
        mUpdateVehicleStateTimer.start(mUpdateVehicleStatePeriod_ms);
    }

    QObject::connect(mVESCMotorController.get(), &VESCMotorController::gotStatusValues, [&](double rpm, int tachometer, int tachometer_abs){
       Q_UNUSED(rpm)
       uint32_t ticks = tachometer_abs;
       uint32_t wheelTickMax = 8388607;
       ticks &=  wheelTickMax; // Bits 31..23 are set to zero

       static int previousTachometer = 0;
       bool direction = ((tachometer - previousTachometer) > previousTachometer);
       ticks |= direction << 23;

       // TODO: input to u-blox disabled for now, seems to cause problems (lost fusion mode on F9R) and needs testing/debugging
       // mUbloxRover->writeOdoToUblox(SINGLE_TICK,ticks);

       // TODO: the following should be signal/slot from (not yet implemented) MovementController::updatedOdomPositionAndYaw to CarPositionFuser
       PosPoint tmpOdomPos = mCarState->getPosition(PosType::odom);
       // VESC does not provide timestamp
       tmpOdomPos.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));
       mCarState->setPosition(tmpOdomPos);
       positionFuser.correctPositionAndYawOdom(mCarState, (tachometer - previousTachometer)/mCarMovementController->getSpeedToRPMFactor()*10.0); // Note: essentially divides tachometer by 6, not sure why needed for VESC

       previousTachometer = tachometer;
    });

    // GNSS (with fused IMU when using u-blox F9R)
    QSharedPointer<UbloxRover> mUbloxRover(new UbloxRover(mCarState));
    foreach(const QSerialPortInfo &portInfo, QSerialPortInfo::availablePorts()) {
        if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
            if (mUbloxRover->connectSerial(portInfo)) {
                qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();

                mUbloxRover->setIMUOrientationOffset(0.0, 0.0, 270.0);
            }
        }
    }
    QObject::connect(mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw, &positionFuser, &CarPositionFuser::correctPositionAndYawGNSS);

    // --- Autopilot ---
    QSharedPointer<WaypointFollower> mWaypointFollower(new WaypointFollower(mCarMovementController));
    mWaypointFollower->setPurePursuitRadius(1.0);
    mWaypointFollower->setRepeatRoute(false);

    // DepthAI Camera & Follow Point
    DepthAiCamera mDepthAiCamera;
    QObject::connect(&mDepthAiCamera, &DepthAiCamera::closestObject, mWaypointFollower.get(), &WaypointFollower::updateFollowPointInVehicleFrame);

    // TCP/IP communication towards RControlStation
    PacketInterfaceTCPServer mPacketIFServer;
    mPacketIFServer.setVehicleState(mCarState);
    mPacketIFServer.setMovementController(mCarMovementController);
    mPacketIFServer.setUbloxRover(mUbloxRover);
    mPacketIFServer.setWaypointFollower(mWaypointFollower);
    QObject::connect(mVESCMotorController.get(), &VESCMotorController::gotStatusValues, &mPacketIFServer, &PacketInterfaceTCPServer::updateMotorControllerStatus);
    mPacketIFServer.listen();

    // Watchdog that warns when EventLoop is slowed down
    SimpleWatchdog watchdog;

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
