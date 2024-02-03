#include <Arm.h>
#include <rev/cansparkmax.h>
//#include <arms> *flexes, cutely*
void Arm::SetArmPosition (arm_position_t DesiredPosition)
{
m_ArmPosition = DesiredPosition;



}

void Arm::updateArm()
{
    double ArmAngle = 0;

 switch (m_ArmPosition)
    {
        case (GROUND_PICKUP):
        {
            ArmAngle = 0;
            break;
        }
        case (SOURCE):
        {
            ArmAngle = 0.4;
            break;
        }
        case (SPEAKER):
        {
            ArmAngle = 0;
            break;
        }
        case (AMP):
        {
            ArmAngle = 0.5;
            break;
        }
        case (TRAP):
        {
            ArmAngle = 1.0;
            break;
        }
    }

//PIDdly thing
const auto ArmControllOutput = m_ArmPIDController.Calculate(
      units::radian_t{m_ArmEncoder.GetDistance()}, units::radian_t{ArmAngle});
m_ArmMotorLeft.Set(ArmControllOutput);
m_ArmMotorRight.Set(ArmControllOutput);
}