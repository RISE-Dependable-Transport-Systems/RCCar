#include "carpositionfuser.h"

CarPositionFuser::CarPositionFuser(QObject *parent) : QObject(parent)
{

}

// "Fusion" currently means using the GNSS position whenever it is updated
// TODO: use IMU/GNSS orientation, consider GNSS age, weight GNSS yaw depending on speed, clamp IMU when stationary, ...
// TODO: sanity checks, e.g., yaw (and position) should not jump
void CarPositionFuser::correctPositionAndYawGNSS(QSharedPointer<VehicleState> vehicleState, bool fused)
{
    Q_UNUSED(fused)
    PosPoint posGNSS = vehicleState->getPosition(PosType::GNSS);
    PosPoint posIMU = vehicleState->getPosition(PosType::IMU);
    PosPoint posFusedTmp = vehicleState->getPosition(PosType::fused);

    posFusedTmp.setXY(posGNSS.getX(), posGNSS.getY());
    posFusedTmp.setHeight(posGNSS.getHeight());
    posFusedTmp.setYaw(posGNSS.getYaw());

    // simply use last knwon roll and pitch from IMU
    posFusedTmp.setRoll(posIMU.getRoll());
    posFusedTmp.setPitch(posIMU.getPitch());

    // TODO: GNSS position's age should be considered
    vehicleState->setPosition(posFusedTmp);
}
