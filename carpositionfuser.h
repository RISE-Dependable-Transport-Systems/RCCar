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

signals:

};

#endif // CARPOSITIONFUSER_H
