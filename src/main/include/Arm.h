#include <rev/cansparkmax.h>
#include <Constants.h>
#include <frc/DutyCycleEncoder.h>
class Arm
{
void updateArm (/*Arm? I hove those!*/);
//Arm
rev::CANSparkMax m_ArmMotorLeft { ConstantCrap::kArmMotorLeftcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
rev::CANSparkMax m_ArmMotorRight { ConstantCrap::kArmMotorRightcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
frc::DutyCycleEncoder m_ArmEncoder  { ConstantCrap::kArmEncoderDIO };

//Wrist
rev::CANSparkMax m_WristMotor {ConstantCrap::kWristMotorID,rev::CANSparkLowLevel::MotorType::kBrushless};
frc::DutyCycleEncoder m_WristEncoder { ConstantCrap::kWristEncoderDIO };
};