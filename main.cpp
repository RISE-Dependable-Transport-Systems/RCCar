#include <QCoreApplication>
#include <QSerialPortInfo>
#include <QDateTime>
#include "sdvp_qtcommon/legacy/packetinterfacetcpserver.h"
#include "sdvp_qtcommon/carstate.h"
#include "sdvp_qtcommon/carmovementcontroller.h"
#include "sdvp_qtcommon/gnss/ubloxrover.h"
#include "sdvp_qtcommon/waypointfollower.h"
#include "sdvp_qtcommon/vescmotorcontroller.h"
#include "carpositionfuser.h"

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    // --- VehicleState and lower-level control setup ---
    QSharedPointer<CarState> mCarState(new CarState);
    QSharedPointer<CarMovementController> mCarMovementController(new CarMovementController(mCarState));

    // setup and connect VESC
    QSharedPointer<VESCMotorController> mVESCMotorController(new VESCMotorController());
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &portInfo, ports) {
        if (portInfo.description().toLower().replace("-", "").contains("chibios")) { // assumption: Serial device with ChibiOS in description is VESC
            mVESCMotorController->connectSerial(portInfo);
            qDebug() << "VESCMotorController connected to:" << portInfo.systemLocation();
        }
    }
    if (mVESCMotorController->isSerialConnected()) {
        mCarMovementController->setMotorController(mVESCMotorController);
        mCarMovementController->setServoController(mVESCMotorController->getServoController()); // VESC is a special case that can even control the servo
    }

    // --- Positioning setup ---
    // Odometry, TODO: use VESC feedback instead of simulating
    const int mUpdateVehicleStatePeriod_ms = 25;
    QTimer mUpdateVehicleStateTimer;
    QObject::connect(&mUpdateVehicleStateTimer, &QTimer::timeout, [&](){
        mCarState->simulationStep(mUpdateVehicleStatePeriod_ms, PosType::odom);
    });
    mUpdateVehicleStateTimer.start(mUpdateVehicleStatePeriod_ms);

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

    // GNSS, with optional IMU on u-blox F9R
    QSharedPointer<UbloxRover> mUbloxRover(new UbloxRover(mCarState));
    foreach(const QSerialPortInfo &portInfo, ports) {
        if (portInfo.manufacturer().toLower().replace("-", "").contains("ublox")) {
            mUbloxRover->connectSerial(portInfo);
            qDebug() << "UbloxRover connected to:" << portInfo.systemLocation();
        }
    }
    mUbloxRover->setEnableIMUOrientationUpdate(!useVESCIMU);

    // Fuse position
    CarPositionFuser positionFuser;
    QObject::connect(mUbloxRover.get(), &UbloxRover::updatedGNSSPositionAndYaw, &positionFuser, &CarPositionFuser::correctPositionAndYawGNSS);

    // --- Autopilot ---
    QSharedPointer<WaypointFollower> mWaypointFollower(new WaypointFollower(mCarMovementController));

    // TCP/IP communication towards RControlStation
    PacketInterfaceTCPServer mPacketIFServer;
    mPacketIFServer.setVehicleState(mCarState);
    mPacketIFServer.setMovementController(mCarMovementController);
    mPacketIFServer.setUbloxRover(mUbloxRover);
    mPacketIFServer.setWaypointFollower(mWaypointFollower);
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
