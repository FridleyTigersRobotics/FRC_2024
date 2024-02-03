#include <rev/cansparkmax.h>
#include <Constants.h>
#include <frc/DutyCycleEncoder.h>
class Arm
{
void updateArm (/*Arm? I hove those!*/);

rev::CANSparkMax m_ArmMotorLeft { ConstantCrap::kArmMotorLeftcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
rev::CANSparkMax m_ArmMotorRight { ConstantCrap::kArmMotorRightcanID,rev::CANSparkLowLevel::MotorType::kBrushless };
frc::DutyCycleEncoder encoder  { 0 };

};