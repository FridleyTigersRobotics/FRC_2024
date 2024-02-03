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



class Arm
{
    arm_position_t m_ArmPosition {GROUND_PICKUP};
void SetArmPosition (arm_position_t DesiredPosition);
void updateArm (/*Arm? I have those!*/);
//Arm
rev::CANSparkMax m_ArmMotorLeft { ConstantCrap::kArmMotorLeftcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
rev::CANSparkMax m_ArmMotorRight { ConstantCrap::kArmMotorRightcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
frc::DutyCycleEncoder m_ArmEncoder  { ConstantCrap::kArmEncoderDIO };

//Wrist
rev::CANSparkMax m_WristMotor {ConstantCrap::kWristMotorID,rev::CANSparkLowLevel::MotorType::kBrushless};
frc::DutyCycleEncoder m_WristEncoder { ConstantCrap::kWristEncoderDIO };

//PID
      static constexpr auto kArmAngleVelocity =
      std::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kArmAngleAcceleration =
      std::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2  
     
      frc::ProfiledPIDController<units::radians> m_ArmPIDController{
      1.0,
      0.0,
      0.0,
      {kArmAngleVelocity, kArmAngleAcceleration}};


};