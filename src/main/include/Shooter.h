#pragma once

#include <rev/cansparkmax.h>
#include <Constants.h>
class Shooter
{
public:
    void updateShooter ( bool spinUpShooter );
    rev::CANSparkMax m_ShooterMotor { ConstantCrap::kShooterMotorID,rev::CANSparkLowLevel::MotorType::kBrushless };

};