#pragma once

namespace ConstantCrap
{
    // Can IDs
    constexpr int kFrontLeftSpinID      = 17;
    constexpr int kFrontLeftDriveID     = 16;
    constexpr int kBackLeftSpinID       = 15;
    constexpr int kBackLeftDriveID      = 14;
    constexpr int kFrontRightSpinID     = 13;
    constexpr int kFrontRightDriveID    = 12;
    constexpr int kBackRightSpinID      = 11;
    constexpr int kBackRightDriveID     = 10;

    // TODO : Check all these, wrist an motor both 18?
    constexpr int kIntakeMotorcanID     = 22;
    constexpr int kArmMotorLeftcanID    = 21;
    constexpr int kArmMotorRightcanID   = 19;
    constexpr int kWristMotorID         = 18;
    constexpr int kShooterMotorID       = 18; 
    constexpr int kRightClimberMotor    = 23;
    constexpr int kLeftClimberMotor     = 24;

    // DIO IDs
    constexpr int kArmEncoderDIO          = 0;
    constexpr int kWristEncoderDIO        = 1;
    constexpr int kRightClimberEncoderDIO = 3;
    constexpr int kLeftClimberEncoderDIO  = 4;

    // Analog IDs
    constexpr int kDriveEncoderBackRight  = 0;
    constexpr int kDriveEncoderFrontRight = 1;
    constexpr int kDriveEncoderBackLeft   = 2;
    constexpr int kDriveEncoderFrontLeft  = 3;


}