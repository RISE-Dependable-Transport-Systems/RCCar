#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QDateTime>
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
    // Odometry, TODO: use VESC feedback instead of simulating
    const int mUpdateVehicleStatePeriod_ms = 25;
    QTimer mUpdateVehicleStateTimer;
    QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [&](){
        mCarState->simulationStep(mUpdateVehicleStatePeriod_ms, (isSimulation ? PosType::fused : PosType::odom));
    });
    mUpdateVehicleStateTimer.start(mUpdateVehicleStatePeriod_ms);

    QObject::connect(mVESCMotorController.get(), &VESCMotorController::gotStatusValues, [&](int tachometer, int tachometer_abs){
       uint32_t ticks = tachometer_abs;
       uint32_t wheel_tick_max = 8388607;
       ticks &=  wheel_tick_max; // Bits 23..31 are set to zero

       static int previous_tachometer = 0;
       bool direction = ((tachometer - previous_tachometer) > previous_tachometer);
       previous_tachometer = tachometer;
       ticks |= direction << 23;

       mUbloxRover->writeOdoToUblox(SINGLE_TICK,ticks);
    });

    bool useVESCIMU = false; // use either VESC or Ublox IMU
    // Optional IMU from VESC
    QObject::connect(mVESCMotorController.get(), &VESCMotorController::gotIMUOrientation, [&](double roll, double pitch, double yaw){
        PosPoint tmpIMUPos = mCarState->getPosition(PosType::IMU);

        tmpIMUPos.setRoll(roll);
        tmpIMUPos.setPitch(pitch);
        tmpIMUPos.setYaw(yaw);
        // VESC does not provide timestamp
        tmpIMUPos.setTime(QTime::currentTime().addSecs(-QDateTime::currentDateTime().offsetFromUtc()).msecsSinceStartOfDay());

        mCarState->setPosition(tmpIMUPos);
    });
    mVESCMotorController->setEnableIMUOrientationUpdate(useVESCIMU);

    // Fuse position
    CarPositionFuser positionFuser;
    QObject::connect(mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw, &positionFuser, &CarPositionFuser::correctPositionAndYawGNSS);

    // --- Autopilot ---
    QSharedPointer<WaypointFollower> mWaypointFollower(new WaypointFollower(mCarMovementController));
    mWaypointFollower->setPurePursuitRadius(3.0);

    // DepthAI Camera
    DepthAiCamera mDepthAiCamera;
    QObject::connect(&mDepthAiCamera, &DepthAiCamera::closestObject, mWaypointFollower.get(), &WaypointFollower::updateFollowMePoint);

    // TCP/IP communication towards RControlStation
    PacketInterfaceTCPServer mPacketIFServer;
    mPacketIFServer.setVehicleState(mCarState);
    mPacketIFServer.setMovementController(mCarMovementController);
    mPacketIFServer.setUbloxRover(mUbloxRover);
    mPacketIFServer.setWaypointFollower(mWaypointFollower);
    QObject::connect(mVESCMotorController.get(), &VESCMotorController::gotStatusValues, &mPacketIFServer, &PacketInterfaceTCPServer::updateMotorControllerStatus);
    mPacketIFServer.listen();

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
