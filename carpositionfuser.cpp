#include "carpositionfuser.h"

CarPositionFuser::CarPositionFuser(QObject *parent) : QObject(parent)
{

}

// "Fusion" currently means using the GNSS position whenever it is updated
// TODO: consider GNSS age, use GNSS yaw for IMU offset and weigh depending on speed, clamp IMU when stationary, ...
// TODO: sanity checks, e.g., yaw (and position) should not jump
void CarPositionFuser::correctPositionAndYawGNSS(QSharedPointer<VehicleState> vehicleState, bool fused)
{
    PosPoint posGNSS = vehicleState->getPosition(PosType::GNSS);
    PosPoint posIMU = vehicleState->getPosition(PosType::IMU);
    PosPoint posFusedTmp = vehicleState->getPosition(PosType::fused);

    posFusedTmp.setXY(posGNSS.getX(), posGNSS.getY());
    posFusedTmp.setHeight(posGNSS.getHeight());

    if (fused) // use GNSS yaw directly if that was already fused (e.g., F9R). TODO: update this with unfused IMU data as well (higher update rate)?
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
