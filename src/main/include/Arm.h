#include <rev/cansparkmax.h>
#include <Constants.h>
#include <frc/DutyCycleEncoder.h>

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
};