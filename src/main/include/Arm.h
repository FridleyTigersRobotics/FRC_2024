#pragma once

#include <rev/cansparkmax.h>
#include <Constants.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <numbers>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

typedef enum arm_positon_e
{
    GROUND_PICKUP,
    SOURCE,
    AMP,
    SPEAKER,
    TRAP
} arm_position_t;

typedef enum wrist_positon_e
{
    WRIST_GROUND_PICKUP,
    WRIST_SOURCE,
    WRIST_SHOOT,
    WRIST_AMP,
    WRIST_TRAP
} wrist_position_t;

class Arm
{
public:
    void initArm();
    void SetArmPosition (arm_position_t DesiredPosition);
    void updateArm (/*Arm? I have those!*/);

private:
    arm_position_t m_ArmPosition {arm_position_t::GROUND_PICKUP};
    wrist_position_t m_WristPosition {wrist_position_t::WRIST_GROUND_PICKUP};

    // Arm
    rev::CANSparkMax m_ArmMotorLeft { ConstantCrap::kArmMotorLeftcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
    rev::CANSparkMax m_ArmMotorRight { ConstantCrap::kArmMotorRightcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
    frc::DutyCycleEncoder m_ArmEncoder  { ConstantCrap::kArmEncoderDIO };

    // Wrist
    rev::CANSparkMax m_WristMotor {ConstantCrap::kWristMotorID,rev::CANSparkLowLevel::MotorType::kBrushless};
    frc::DutyCycleEncoder m_WristEncoder { ConstantCrap::kWristEncoderDIO };

    // ARM PID
    static constexpr auto kArmAngleVelocity =
      std::numbers::pi * 1_rad_per_s;  // radians per second
    static constexpr auto kArmAngleAcceleration =
      std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2  
     
    frc::ProfiledPIDController<units::radians> m_ArmPIDController{
      1.0,
      0.0,
      0.0,
      {kArmAngleVelocity, kArmAngleAcceleration}};

    // WRIST PID
    static constexpr auto kWristAngleVelocity =
      std::numbers::pi * 1_rad_per_s;  // radians per second
    static constexpr auto kWristAngleAcceleration =
      std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2  
     
    frc::ProfiledPIDController<units::radians> m_WristPIDController{
      1.0,
      0.0,
      0.0,
      {kWristAngleVelocity, kWristAngleAcceleration}};

};