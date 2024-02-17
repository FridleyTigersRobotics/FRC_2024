#pragma once

#include "rev/CANSparkMax.h"
#include <Constants.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/math.h>
#include <units/time.h>

class Shooter
{
public:
    void initShooter();
    void updateShooter ( bool spinUpShooter );
    bool ReadyToShoot();

private:
    rev::CANSparkMax          m_shooterMotor   { ConstantCrap::kShooterMotorID,rev::CANSparkLowLevel::MotorType::kBrushless };
    rev::SparkPIDController   m_shooterPid     { m_shooterMotor.GetPIDController() };
    rev::SparkRelativeEncoder m_shooterEncoder { m_shooterMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42) };

    // TODO : Determine the acceleration.
    frc::SlewRateLimiter<units::scalar> m_AccelerationLimiter{1 / 1_s};

    bool   m_shooterSpeedReadyToShoot { false };

    // TODO : Determine all thse values.
    double const m_maxShooterSpeed{ 2000 };
    double const m_maxAccelOutput { 0.8 };
};