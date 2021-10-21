#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QDateTime>
#include "sdvp_qtcommon/simplewatchdog.h"
#include "sdvp_qtcommon/legacy/packetinterfacetcpserver.h"
#include "sdvp_qtcommon/carstate.h"
#include "sdvp_qtcommon/carmovementcontroller.h"
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

    // GNSS, with optional IMU on u-blox F9R
    QSharedPointer<UbloxRover> mUbloxRover(new UbloxRover(mCarState));
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &portInfo, ports) {
        if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
            if (mUbloxRover->connectSerial(portInfo)) {
                qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();

                mUbloxRover->setIMUOrientationOffset(0.0, 0.0, 270.0);
                mUbloxRover->setEnableIMUOrientationUpdate(false); // Whether to use raw IMU data from F9R
            }
        }
    }

    // setup and connect VESC
    QSharedPointer<VESCMotorController> mVESCMotorController(new VESCMotorController());
    foreach(const QSerialPortInfo &portInfo, ports) {
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
    // Odometry
    const int mUpdateVehicleStatePeriod_ms = 25;
    QTimer mUpdateVehicleStateTimer;
    QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [&](){
        mCarState->simulationStep(mUpdateVehicleStatePeriod_ms, (isSimulation ? PosType::fused : PosType::odom));
    });
    mUpdateVehicleStateTimer.start(mUpdateVehicleStatePeriod_ms);

    QObject::connect(mVESCMotorController.get(), &VESCMotorController::gotStatusValues, [&](double rpm, int tachometer, int tachometer_abs){
       Q_UNUSED(rpm)
       uint32_t ticks = tachometer_abs;
       uint32_t wheel_tick_max = 8388607;
       ticks &=  wheel_tick_max; // Bits 23..31 are set to zero

       static int previous_tachometer = 0;
       bool direction = ((tachometer - previous_tachometer) > previous_tachometer);
       previous_tachometer = tachometer;
       ticks |= direction << 23;

       mUbloxRover->writeOdoToUblox(SINGLE_TICK,ticks);
    });

    // Take roll & pitch, yaw from VESC's IMU
    QObject::connect(mVESCMotorController.get(), &VESCMotorController::gotIMUOrientation, [&](double roll, double pitch, double yaw){
        PosPoint tmpIMUPos = mCarState->getPosition(PosType::IMU);

        tmpIMUPos.setRoll(roll);
        tmpIMUPos.setPitch(pitch);
        tmpIMUPos.setYaw(yaw);

        // VESC does not provide timestamp
        tmpIMUPos.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));

        mCarState->setPosition(tmpIMUPos);
    });
    mVESCMotorController->setEnableIMUOrientationUpdate(true);

    // Fuse position
    CarPositionFuser positionFuser;
    QObject::connect(mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw, &positionFuser, &CarPositionFuser::correctPositionAndYawGNSS);

    // --- Autopilot ---
    QSharedPointer<WaypointFollower> mWaypointFollower(new WaypointFollower(mCarMovementController));
    mWaypointFollower->setPurePursuitRadius(1.0);
    mWaypointFollower->setRepeatRoute(true);

    // DepthAI Camera & Follow Point
    DepthAiCamera mDepthAiCamera;
    QObject::connect(&mDepthAiCamera, &DepthAiCamera::closestObject, [&](const PosPoint &obj){
        // 1. transform obj from vehicle frame to ENU frame
        // TODO: Take offset from camera to middle of backaxle into account
        PosPoint carPosition = mCarState->getPosition(PosType::fused);

        // clockwise rotation
        double currYaw_rad = (carPosition.getYaw() + 90.0) * (M_PI / 180.0);
        double newX = cos(currYaw_rad)*obj.getX() + sin(currYaw_rad)*obj.getY();
        double newY = -sin(currYaw_rad)*obj.getX() + cos(currYaw_rad)*obj.getY();

        // translation
        newX += carPosition.getX();
        newY += carPosition.getY();

        PosPoint pointInEnuFrame;
        pointInEnuFrame.setX(newX);
        pointInEnuFrame.setY(newY);

        // set timestamp
        pointInEnuFrame.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()));

        // 2. use point for follow point function
        mWaypointFollower->updateFollowPoint(pointInEnuFrame);
    });

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
