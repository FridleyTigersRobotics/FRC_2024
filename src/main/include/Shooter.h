#pragma once

#include <rev/cansparkmax.h>
#include <Constants.h>
class Shooter
{
void updateShooter (/*among us*/);
public:
rev::CANSparkMax m_ShooterMotor { ConstantCrap::kShooterMotorID,rev::CANSparkLowLevel::MotorType::kBrushless };

};