#include "carpositionfuser.h"

CarPositionFuser::CarPositionFuser(QObject *parent) : QObject(parent)
{

}

// "Fusion" currently means using the GNSS position whenever it is updated
// TODO: consider GNSS age, use GNSS yaw for IMU offset and weigh depending on speed/distance, ...
// TODO: sanity checks, e.g., yaw (and position) should not jump
void CarPositionFuser::correctPositionAndYawGNSS(QSharedPointer<VehicleState> vehicleState, bool fused)
{
    PosPoint posGNSS = vehicleState->getPosition(PosType::GNSS);
    PosPoint posIMU = vehicleState->getPosition(PosType::IMU);
    PosPoint posFusedTmp = vehicleState->getPosition(PosType::fused);

    // TODO "fusion" of GNSS and IMU-based dead reckoning
    posFusedTmp.setXY(posGNSS.getX(), posGNSS.getY());
    posFusedTmp.setHeight(posGNSS.getHeight());

    if (fused) // use GNSS yaw directly if that was already fused (e.g., F9R).
        posFusedTmp.setYaw(posGNSS.getYaw());
    else {
        // TODO: offset by GNSS yaw to correct drift
        posFusedTmp.setYaw(posIMU.getYaw());
    }

    // simply use last known roll and pitch from IMU
    posFusedTmp.setRoll(posIMU.getRoll());
    posFusedTmp.setPitch(posIMU.getPitch());

    // TODO: GNSS position's age should be considered
    vehicleState->setPosition(posFusedTmp);
}

void CarPositionFuser::correctPositionAndYawOdom(QSharedPointer<VehicleState> vehicleState, double distanceDriven)
{
    // use Odom input from motorcontroller for IMU-based dead reckoning
    PosPoint posIMU = vehicleState->getPosition(PosType::IMU);
    double yawRad = posIMU.getYaw() / (180.0/M_PI);
    posIMU.setXY(posIMU.getX() + cos(-yawRad) * distanceDriven,
                 posIMU.getY() + sin(-yawRad) * distanceDriven);
    // TODO: write PosType::fused when GNSS not in fused mode (F9R)
    vehicleState->setPosition(posIMU);
}

void CarPositionFuser::correctPositionAndYawIMU(QSharedPointer<VehicleState> vehicleState)
{
    static bool standstillAtLastCall = false;
    static double yawWhenStopping = 0.0;
    static double yawDriftSinceStandstill = 0.0;

    // --- correct relative/raw IMU yaw with external offset
    PosPoint posIMU = vehicleState->getPosition(PosType::IMU);

    // 1. handle drift at standstill
    if (fabs(vehicleState->getSpeed()) < 0.05) {
        if (!standstillAtLastCall)
            yawWhenStopping = posIMU.getYaw();

        standstillAtLastCall = true;
        yawDriftSinceStandstill = yawWhenStopping - posIMU.getYaw();
        posIMU.setYaw(yawWhenStopping); // fix yaw during standstill
    } else {
        if (standstillAtLastCall)
            mPosIMUyawOffset += yawDriftSinceStandstill;

        standstillAtLastCall = false;
    }

    // 2. apply offset
    // TODO: normalization, write PosType::fused when GNSS not in fused mode (F9R)
    posIMU.setYaw(posIMU.getYaw() + mPosIMUyawOffset);
    vehicleState->setPosition(posIMU);
}
