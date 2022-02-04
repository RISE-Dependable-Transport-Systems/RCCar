#ifndef CARPOSITIONFUSER_H
#define CARPOSITIONFUSER_H

#include <QObject>
#include <QSharedPointer>
#include <sdvp_qtcommon/vehiclestate.h>

class CarPositionFuser : public QObject
{
    Q_OBJECT
public:
    explicit CarPositionFuser(QObject *parent = nullptr);
    void correctPositionAndYawGNSS(QSharedPointer<VehicleState> vehicleState, bool fused);
    void correctPositionAndYawOdom(QSharedPointer<VehicleState> vehicleState, double distanceDriven);
    void correctPositionAndYawIMU(QSharedPointer<VehicleState> vehicleState);

signals:

private:
    double mPosIMUyawOffset = 0.0;
};

#endif // CARPOSITIONFUSER_H
